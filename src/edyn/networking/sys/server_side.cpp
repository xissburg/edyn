#include "edyn/networking/sys/server_side.hpp"
#include "edyn/networking/packet/client_created.hpp"
#include "edyn/networking/packet/transient_snapshot.hpp"
#include "edyn/networking/packet/update_entity_map.hpp"
#include "edyn/networking/packet/util/pool_snapshot.hpp"
#include "edyn/networking/comp/remote_client.hpp"
#include "edyn/networking/comp/aabb_of_interest.hpp"
#include "edyn/networking/comp/entity_owner.hpp"
#include "edyn/networking/sys/update_aabbs_of_interest.hpp"
#include "edyn/networking/context/server_networking_context.hpp"
#include "edyn/networking/util/server_import_pool.hpp"
#include "edyn/parallel/merge/merge_component.hpp"
#include "edyn/time/time.hpp"
#include "edyn/util/entity_map.hpp"
#include "edyn/edyn.hpp"
#include "edyn/util/vector.hpp"
#include <entt/core/type_traits.hpp>
#include <entt/entity/registry.hpp>
#include <algorithm>
#include <set>

namespace edyn {

bool is_fully_owned_by_client(const entt::registry &registry, entt::entity client_entity, entt::entity entity) {
    auto &resident = registry.get<island_resident>(entity);
    auto &island = registry.get<edyn::island>(resident.island_entity);
    auto owned_by_this_client = false;
    auto owned_by_another_client = false;

    for (auto node : island.nodes) {
        if (auto *owner = registry.try_get<entity_owner>(node)) {
            if (owner->client_entity == client_entity) {
                owned_by_this_client = true;
            } else {
                owned_by_another_client = true;
            }
        }
    }

    return owned_by_this_client && !owned_by_another_client;
}

static void process_packet(entt::registry &registry, entt::entity client_entity, const packet::entity_request &req) {
    auto &ctx = registry.ctx<server_networking_context>();
    auto res = packet::entity_response{};
    auto entities = std::set<entt::entity>(req.entities.begin(), req.entities.end());

    for (auto entity : entities) {
        if (!registry.valid(entity)) {
            continue;
        }

        res.entities.push_back(entity);
        (*ctx.insert_entity_components_func)(registry, entity, res.pools);
    }

    if (!res.entities.empty()) {
        std::sort(res.pools.begin(), res.pools.end(), [] (auto &&lhs, auto &&rhs) {
            return lhs.component_index < rhs.component_index;
        });

        auto &client = registry.get<remote_client>(client_entity);
        client.packet_signal.publish(client_entity, packet::edyn_packet{res});
    }
}

static void process_packet(entt::registry &registry, entt::entity client_entity, const packet::entity_response &res) {

}

static void process_packet(entt::registry &registry, entt::entity client_entity, const packet::transient_snapshot &snapshot) {
    auto &ctx = registry.ctx<server_networking_context>();

    for (auto &pool : snapshot.pools) {
        (*ctx.import_pool_func)(registry, client_entity, pool, false);
    }
}

static void process_packet(entt::registry &registry, entt::entity client_entity, const packet::general_snapshot &snapshot) {
    auto &ctx = registry.ctx<server_networking_context>();

    for (auto &pool : snapshot.pools) {
        (*ctx.import_pool_func)(registry, client_entity, pool, true);
    }
}

template<typename T>
void create_graph_edge(entt::registry &registry, entt::entity entity) {
    if (registry.any_of<graph_edge>(entity)) return;

    auto &comp = registry.get<T>(entity);
    auto node_index0 = registry.get<graph_node>(comp.body[0]).node_index;
    auto node_index1 = registry.get<graph_node>(comp.body[1]).node_index;
    auto edge_index = registry.ctx<entity_graph>().insert_edge(entity, node_index0, node_index1);
    registry.emplace<graph_edge>(entity, edge_index);
}

template<typename... Ts>
void maybe_create_graph_edge(entt::registry &registry, entt::entity entity) {
    ((registry.any_of<Ts>(entity) ? create_graph_edge<Ts>(registry, entity) : void(0)), ...);
}

static void process_packet(entt::registry &registry, entt::entity client_entity, const packet::create_entity &packet) {
    auto &ctx = registry.ctx<server_networking_context>();
    auto &client = registry.get<remote_client>(client_entity);

    // Collect entity mappings for new entities to send back to client.
    auto emap_packet = packet::update_entity_map{};

    for (auto remote_entity : packet.entities) {
        if (client.entity_map.has_rem(remote_entity)) continue;

        auto local_entity = registry.create();
        registry.emplace<entity_owner>(local_entity, client_entity);

        emap_packet.pairs.emplace_back(remote_entity, local_entity);
        client.entity_map.insert(remote_entity, local_entity);
        client.owned_entities.push_back(local_entity);
    }

    if (!emap_packet.pairs.empty()) {
        client.packet_signal.publish(client_entity, packet::edyn_packet{emap_packet});
    }

    for (auto &pool : packet.pools) {
        (*ctx.import_pool_func)(registry, client_entity, pool, false);
    }

    for (auto remote_entity : packet.entities) {
        auto local_entity = client.entity_map.remloc(remote_entity);
        registry.emplace<networked_tag>(local_entity);

        /* auto [null_entity, null_con] = edyn::make_constraint<edyn::null_constraint>(registry, local_entity, client_entity);
        registry.emplace<edyn::entity_owner>(null_entity, client_entity);
        registry.emplace<edyn::networked_tag>(null_entity); */
    }

    // Create nodes and edges in entity graph.
    for (auto remote_entity : packet.entities) {
        auto local_entity = client.entity_map.remloc(remote_entity);

        if (registry.any_of<rigidbody_tag, external_tag>(local_entity) &&
            !registry.all_of<graph_node>(local_entity)) {
            auto non_connecting = !registry.any_of<procedural_tag>(local_entity);
            auto node_index = registry.ctx<entity_graph>().insert_node(local_entity, non_connecting);
            registry.emplace<graph_node>(local_entity, node_index);
        }
    }

    for (auto remote_entity : packet.entities) {
        auto local_entity = client.entity_map.remloc(remote_entity);

        maybe_create_graph_edge<
            contact_manifold,
            null_constraint,
            gravity_constraint,
            point_constraint,
            distance_constraint,
            soft_distance_constraint,
            hinge_constraint,
            generic_constraint
        >(registry, local_entity);
    }
}

static void process_packet(entt::registry &registry, entt::entity client_entity, const packet::destroy_entity &packet) {
    auto &client = registry.get<remote_client>(client_entity);

    for (auto remote_entity : packet.entities) {
        if (client.entity_map.has_rem(remote_entity)) {
            auto local_entity = client.entity_map.remloc(remote_entity);

            if (auto *owner = registry.try_get<entity_owner>(local_entity);
                owner && owner->client_entity == client_entity) {
                registry.destroy(local_entity);
            }

            client.entity_map.erase_rem(remote_entity);
            vector_erase(client.owned_entities, local_entity);
        }
    }
}

static void process_packet(entt::registry &registry, entt::entity client_entity, const packet::update_entity_map &packet) {
    auto &client = registry.get<remote_client>(client_entity);

    for (auto &pair : packet.pairs) {
        auto local_entity = pair.first;
        auto remote_entity = pair.second;
        client.entity_map.insert(remote_entity, local_entity);
    }
}

static void handle_packet(entt::registry &registry, entt::entity client_entity, const packet::entity_request &req) {
    process_packet(registry, client_entity, req);
}

static void handle_packet(entt::registry &registry, entt::entity client_entity, const packet::entity_response &res) {
    process_packet(registry, client_entity, res);
}

static void handle_packet(entt::registry &registry, entt::entity client_entity, const packet::transient_snapshot &snapshot) {
    auto &client = registry.get<remote_client>(client_entity);
    auto timestamp = performance_time() - client.latency;
    auto packet = client_packet{packet::edyn_packet{snapshot}, timestamp};
    client.packet_queue.push_back(packet);
}

static void handle_packet(entt::registry &registry, entt::entity client_entity, const packet::general_snapshot &snapshot) {
    auto &client = registry.get<remote_client>(client_entity);
    auto timestamp = performance_time() - client.latency;
    auto packet = client_packet{packet::edyn_packet{snapshot}, timestamp};
    client.packet_queue.push_back(packet);
}

static void handle_packet(entt::registry &registry, entt::entity client_entity, const packet::create_entity &create) {
    auto &client = registry.get<remote_client>(client_entity);
    auto timestamp = performance_time() - client.latency;
    auto packet = client_packet{packet::edyn_packet{create}, timestamp};
    client.packet_queue.push_back(packet);
}

static void handle_packet(entt::registry &registry, entt::entity client_entity, const packet::destroy_entity &destroy) {
    auto &client = registry.get<remote_client>(client_entity);
    auto timestamp = performance_time() - client.latency;
    auto packet = client_packet{packet::edyn_packet{destroy}, timestamp};
    client.packet_queue.push_back(packet);
}

static void handle_packet(entt::registry &registry, entt::entity client_entity, const packet::update_entity_map &packet) {
    process_packet(registry, client_entity, packet);
}

static void handle_packet(entt::registry &, entt::entity, const packet::client_created &) {}
static void process_packet(entt::registry &, entt::entity, const packet::client_created &) {}

void init_networking_server(entt::registry &registry) {
    registry.set<server_networking_context>();
}

void deinit_networking_server(entt::registry &registry) {
    registry.unset<server_networking_context>();
}

void server_process_packets(entt::registry &registry) {
    auto *ctx = registry.try_ctx<server_networking_context>();
    if (!ctx) return;

    auto timestamp = performance_time();

    registry.view<remote_client>().each([&] (entt::entity client_entity, remote_client &client) {
        auto first = client.packet_queue.begin();
        auto last = std::find_if(first, client.packet_queue.end(), [&] (auto &&packet) {
            return packet.arrival_timestamp > timestamp - client.playout_delay;
        });

        for (auto it = first; it != last; ++it) {
            std::visit([&] (auto &&decoded_packet) {
                process_packet(registry, client_entity, decoded_packet);
            }, it->packet.var);
        }

        client.packet_queue.erase(first, last);
    });
}

void update_networking_server(entt::registry &registry) {
    auto *ctx = registry.try_ctx<server_networking_context>();
    if (!ctx) return;

    auto time = performance_time();

    server_process_packets(registry);

    update_aabbs_of_interest(registry);

    auto view = registry.view<remote_client, aabb_of_interest>();
    view.each([&] (entt::entity client_entity, remote_client &client, aabb_of_interest &aabboi) {
        if (!aabboi.destroy_entities.empty()) {
            auto packet = packet::destroy_entity{};

            for (auto entity : aabboi.destroy_entities) {
                if (!registry.valid(entity)) continue;

                if (auto *owner = registry.try_get<entity_owner>(entity);
                    owner == nullptr || owner->client_entity != client_entity) {
                    packet.entities.push_back(entity);
                }
            }

            if (!packet.entities.empty()) {
                client.packet_signal.publish(client_entity, packet::edyn_packet{packet});
            }

            aabboi.destroy_entities.clear();
        }

        if (!aabboi.create_entities.empty()) {
            auto packet = packet::create_entity{};

            for (auto entity : aabboi.create_entities) {
                if (auto *owner = registry.try_get<entity_owner>(entity);
                    owner == nullptr || owner->client_entity != client_entity) {
                    packet.entities.push_back(entity);
                }
            }

            if (!packet.entities.empty()) {
                for (auto entity : packet.entities) {
                    (*ctx->insert_entity_components_func)(registry, entity, packet.pools);
                }

                std::sort(packet.pools.begin(), packet.pools.end(), [] (auto &&lhs, auto &&rhs) {
                    return lhs.component_index < rhs.component_index;
                });

                client.packet_signal.publish(client_entity, packet::edyn_packet{packet});
            }

            aabboi.create_entities.clear();
        }

        if (time - client.last_snapshot_time > 1 / client.snapshot_rate) {
            auto packet = packet::transient_snapshot{};

            for (auto entity : aabboi.entities) {
                if (!registry.all_of<procedural_tag, networked_tag>(entity)) {
                    continue;
                }

                // Only include entities which are in islands not fully owned by the client.
                if (!is_fully_owned_by_client(registry, client_entity, entity)) {
                   (*ctx->insert_transient_components_func)(registry, entity, packet.pools);
                }
            }

            client.last_snapshot_time = time;

            if (!packet.pools.empty()) {
                client.packet_signal.publish(client_entity, packet::edyn_packet{packet});
            }
        }
    });

    for (auto client_entity : ctx->pending_created_clients) {
        auto &client = registry.get<remote_client>(client_entity);
        auto packet = packet::client_created{client_entity};
        client.packet_signal.publish(client_entity, packet::edyn_packet{packet});
    }

    ctx->pending_created_clients.clear();

    // Send out accumulated changes to clients.
    registry.view<remote_client>().each([&] (entt::entity client_entity, remote_client &client) {
        if (client.current_snapshot.pools.empty()) return;
        auto packet = packet::edyn_packet{std::move(client.current_snapshot)};
        client.packet_signal.publish(client_entity, packet);
        EDYN_ASSERT(client.current_snapshot.pools.empty());
    });
}

void server_handle_packet(entt::registry &registry, entt::entity client_entity, const packet::edyn_packet &packet) {
    std::visit([&] (auto &&decoded_packet) {
        handle_packet(registry, client_entity, decoded_packet);
    }, packet.var);
}

void server_make_client(entt::registry &registry, entt::entity entity) {
    registry.emplace<remote_client>(entity);
    registry.emplace<aabb_of_interest>(entity);
    edyn::tag_external_entity(registry, entity, false);

    auto &ctx = registry.ctx<server_networking_context>();
    ctx.pending_created_clients.push_back(entity);
}

entt::entity server_make_client(entt::registry &registry) {
    auto entity = registry.create();
    server_make_client(registry, entity);
    return entity;
}

void server_set_client_latency(entt::registry &registry, entt::entity client_entity, double latency) {
    auto &client = registry.get<remote_client>(client_entity);
    client.latency = latency;
    client.playout_delay = latency * 1.2;
}

}
