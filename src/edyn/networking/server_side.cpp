#include "edyn/networking/server_side.hpp"
#include "edyn/networking/packet/client_created.hpp"
#include "edyn/networking/packet/transient_snapshot.hpp"
#include "edyn/networking/packet/update_entity_map.hpp"
#include "edyn/networking/packet/util/pool_snapshot.hpp"
#include "edyn/networking/remote_client.hpp"
#include "edyn/networking/aabb_of_interest.hpp"
#include "edyn/networking/entity_owner.hpp"
#include "edyn/networking/update_aabbs_of_interest.hpp"
#include "edyn/networking/server_networking_context.hpp"
#include "edyn/networking/server_import_pool.hpp"
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

template<typename Component>
void import_pool(entt::registry &registry, entt::entity client_entity,
                 const std::vector<std::pair<entt::entity, Component>> &pool) {
    if constexpr(std::is_empty_v<Component>) {
        return;
    }

    auto &client = registry.get<remote_client>(client_entity);

    for (auto &pair : pool) {
        auto remote_entity = pair.first;

        if (!client.entity_map.has_rem(pair.first)) {
            continue;
        }

        auto local_entity = client.entity_map.remloc(remote_entity);

        if (!registry.valid(local_entity)) {
            client.entity_map.erase_loc(local_entity);
            continue;
        }

        // Do not apply this update if this is a dynamic entity which is not
        // fully owned by this client.
        if (registry.any_of<dynamic_tag>(local_entity) && !is_fully_owned_by_client(registry, client_entity, local_entity)) {
            continue;
        }

        if (registry.any_of<Component>(local_entity)) {
            registry.replace<Component>(local_entity, pair.second);
            edyn::refresh<Component>(registry, local_entity);
        } else {
            registry.emplace<Component>(local_entity, pair.second);
            registry.emplace_or_replace<dirty>(local_entity).template created<Component>();
        }
    }
}

static void process_packet(entt::registry &registry, entt::entity client_entity, const packet::transient_snapshot &snapshot) {
    auto &ctx = registry.ctx<server_networking_context>();

    for (auto &pool : snapshot.pools) {
        (*ctx.import_pool_func)(registry, client_entity, pool);
    }
}


static void process_packet(entt::registry &registry, entt::entity client_entity, const packet::create_entity &packet) {
    auto &ctx = registry.ctx<server_networking_context>();
    auto &client = registry.get<remote_client>(client_entity);
    auto emap_packet = packet::update_entity_map{};

    for (auto remote_entity : packet.entities) {
        auto local_entity = registry.create();
        registry.emplace<entity_owner>(local_entity, client_entity);

        emap_packet.pairs.emplace_back(remote_entity, local_entity);
        client.entity_map.insert(remote_entity, local_entity);
        client.owned_entities.push_back(local_entity);
    }

    client.packet_signal.publish(client_entity, packet::edyn_packet{emap_packet});

    for (auto &pool : packet.pools) {
        (*ctx.import_pool_func)(registry, client_entity, pool);
    }

    for (auto remote_entity : packet.entities) {
        auto local_entity = client.entity_map.remloc(remote_entity);
        registry.emplace<networked_tag>(local_entity);

        /* auto [null_entity, null_con] = edyn::make_constraint<edyn::null_constraint>(registry, local_entity, client_entity);
        registry.emplace<edyn::entity_owner>(null_entity, client_entity);
        registry.emplace<edyn::networked_tag>(null_entity); */
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
    auto packet = packet::edyn_packet{snapshot};
    packet.timestamp = performance_time() - client.latency;
    client.packet_queue.push_back(packet);
}

static void handle_packet(entt::registry &registry, entt::entity client_entity, const packet::create_entity &create) {
    auto &client = registry.get<remote_client>(client_entity);
    auto packet = packet::edyn_packet{create};
    packet.timestamp = performance_time() - client.latency;
    client.packet_queue.push_back(packet);
}

static void handle_packet(entt::registry &registry, entt::entity client_entity, const packet::destroy_entity &destroy) {
    auto &client = registry.get<remote_client>(client_entity);
    auto packet = packet::edyn_packet{destroy};
    packet.timestamp = performance_time() - client.latency;
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
            return packet.timestamp > timestamp - client.playout_delay;
        });

        for (auto it = first; it != last; ++it) {
            std::visit([&] (auto &&decoded_packet) {
                process_packet(registry, client_entity, decoded_packet);
            }, it->var);
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
            packet.entities = std::move(aabboi.destroy_entities);
            client.packet_signal.publish(client_entity, packet::edyn_packet{packet});
        }

        if (!aabboi.create_entities.empty()) {
            auto packet = packet::create_entity{};
            packet.entities = aabboi.create_entities;

            for (auto entity : aabboi.create_entities) {
                (*ctx->insert_entity_components_func)(registry, entity, packet.pools);
            }

            std::sort(packet.pools.begin(), packet.pools.end(), [] (auto &&lhs, auto &&rhs) {
                return lhs.component_index < rhs.component_index;
            });

            client.packet_signal.publish(client_entity, packet::edyn_packet{packet});
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
}

void server_handle_packet(entt::registry &registry, entt::entity client_entity, const packet::edyn_packet &packet) {
    std::visit([&] (auto &&decoded_packet) {
        handle_packet(registry, client_entity, decoded_packet);
    }, packet.var);
}

template<typename Component>
void insert_all_into_pool(entt::registry &registry, entt::entity client_entity, std::vector<std::pair<entt::entity, Component>> &pool) {
    auto view = registry.view<Component, procedural_tag, networked_tag>();

    if (view.size_hint() == 0) {
        return;
    }

    for (auto entity : view) {
        // Only include entities which are in islands not fully owned by the client.
        if (is_fully_owned_by_client(registry, client_entity, entity)) {
            continue;
        }

        if constexpr(std::is_empty_v<Component>) {
            pool.emplace_back(entity);
        } else {
            auto &comp = view.template get<Component>(entity);
            pool.emplace_back(entity, comp);
        }
    }
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
