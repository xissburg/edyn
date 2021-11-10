#include "edyn/networking/server_side.hpp"
#include "edyn/networking/packet/transient_snapshot.hpp"
#include "edyn/networking/packet/update_entity_map.hpp"
#include "edyn/networking/packet/util/pool_snapshot.hpp"
#include "edyn/networking/remote_client.hpp"
#include "edyn/networking/aabb_of_interest.hpp"
#include "edyn/networking/entity_owner.hpp"
#include "edyn/networking/update_aabbs_of_interest.hpp"
#include "edyn/networking/server_networking_context.hpp"
#include "edyn/time/time.hpp"
#include "edyn/util/entity_map.hpp"
#include "edyn/edyn.hpp"
#include "edyn/util/vector.hpp"
#include <entt/core/type_traits.hpp>
#include <entt/entity/registry.hpp>
#include <algorithm>
#include <set>

namespace edyn {

static bool is_fully_owned_by_client(const entt::registry &registry, entt::entity client_entity, entt::entity entity) {
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
    auto res = packet::entity_response{};
    auto entities = std::set<entt::entity>(req.entities.begin(), req.entities.end());

    for (auto entity : entities) {
        if (!registry.valid(entity)) {
            continue;
        }

        auto pair = make_entity_components_pair(registry, entity);
        res.pairs.push_back(pair);
    }

    if (!res.pairs.empty()) {
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
    import_pool(registry, client_entity, snapshot.positions);
    import_pool(registry, client_entity, snapshot.orientations);
    import_pool(registry, client_entity, snapshot.linvels);
    import_pool(registry, client_entity, snapshot.angvels);
}

static void process_packet(entt::registry &registry, entt::entity client_entity, const packet::create_entity &packet) {
    auto &client = registry.get<remote_client>(client_entity);
    auto emap_packet = packet::update_entity_map{};

    for (auto &pair : packet.pairs) {
        auto local_entity = registry.create();
        auto remote_entity = pair.entity;
        registry.emplace<entity_owner>(local_entity, client_entity);

        emap_packet.pairs.emplace_back(remote_entity, local_entity);
        client.entity_map.insert(remote_entity, local_entity);
        client.owned_entities.push_back(local_entity);
    }

    client.packet_signal.publish(client_entity, packet::edyn_packet{emap_packet});

    for (auto &pair : packet.pairs) {
        auto remote_entity = pair.entity;
        auto local_entity = client.entity_map.remloc(remote_entity);

        for (auto &comp_ptr : pair.components) {
            assign_component_wrapper(registry, local_entity, *comp_ptr, client.entity_map);
        }

        registry.emplace<networked_tag>(local_entity);
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

void init_networking_server(entt::registry &registry) {
    registry.set<server_networking_context>();
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

            for (auto entity : aabboi.create_entities) {
                auto pair = make_entity_components_pair(registry, entity);
                packet.pairs.push_back(pair);
            }

            client.packet_signal.publish(client_entity, packet::edyn_packet{packet});
            aabboi.create_entities.clear();
        }
    });
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

packet::transient_snapshot server_get_transient_snapshot(entt::registry &registry, entt::entity client_entity) {
    auto snapshot = packet::transient_snapshot{};
    insert_all_into_pool(registry, client_entity, snapshot.positions);
    insert_all_into_pool(registry, client_entity, snapshot.orientations);
    insert_all_into_pool(registry, client_entity, snapshot.linvels);
    insert_all_into_pool(registry, client_entity, snapshot.angvels);
    return snapshot;
}

void server_make_client(entt::registry &registry, entt::entity entity) {
    registry.emplace<remote_client>(entity);
    registry.emplace<aabb_of_interest>(entity);
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
