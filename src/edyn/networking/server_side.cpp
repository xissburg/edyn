#include "edyn/networking/server_side.hpp"
#include "edyn/networking/packet/transient_snapshot.hpp"
#include "edyn/networking/packet/update_entity_map.hpp"
#include "edyn/networking/remote_client.hpp"
#include "edyn/networking/entity_owner.hpp"
#include "edyn/comp/transient_comp.hpp"
#include "edyn/util/entity_map.hpp"
#include "edyn/edyn.hpp"
#include <entt/core/type_traits.hpp>
#include <entt/entity/registry.hpp>
#include <set>

namespace edyn {

void init_networking_server(entt::registry &) {

}

static void process_packet(entt::registry &registry, entt::entity client_entity, const entity_request &req) {
    auto res = entity_response{};
    auto entities = std::set<entt::entity>(req.entities.begin(), req.entities.end());

    for (auto entity : entities) {
        if (!registry.valid(entity)) {
            continue;
        }

        auto pair = make_entity_components_pair(registry, entity);
        res.pairs.push_back(std::move(pair));
    }

    if (!res.pairs.empty()) {
        auto &client = registry.get<remote_client>(client_entity);
        client.packet_signal.publish(edyn_packet{std::move(res)});
    }
}

static void process_packet(entt::registry &registry, entt::entity client_entity, const entity_response &res) {

}

template<typename Component>
void import_pool(entt::registry &registry, entt::entity client_entity, const pool_snapshot<Component> &pool) {
    if constexpr(entt::is_eto_eligible_v<Component>) {
        return;
    }

    auto &client = registry.get<remote_client>(client_entity);

    for (auto &pair : pool.pairs) {
        auto remote_entity = pair.first;

        if (client.entity_map.has_rem(pair.first)) {
            auto local_entity = client.entity_map.remloc(remote_entity);

            if (registry.valid(local_entity)) {
                if (registry.has<Component>(local_entity)) {
                    registry.replace<Component>(local_entity, pair.second);
                    edyn::refresh<Component>(registry, local_entity);
                } else {
                    registry.emplace<Component>(local_entity, pair.second);
                    registry.emplace_or_replace<dirty>(local_entity).template created<Component>();
                }
            } else {
                client.entity_map.erase_loc(local_entity);
            }
        }
    }
}

void process_pool(entt::registry &registry, entt::entity client_entity, const pool_snapshot_base &pool) {
    std::apply([&] (auto ... comp) {
        size_t i = 0;
        ((i++ == pool.component_index ? import_pool(registry, client_entity, static_cast<const pool_snapshot<decltype(comp)> &>(pool)) : void(0)), ...);
    }, networked_components);
}

static void process_packet(entt::registry &registry, entt::entity client_entity, const transient_snapshot &snapshot) {
    for (auto &pool_ptr : snapshot.pools) {
        process_pool(registry, client_entity, *pool_ptr);
    }
}

static void process_packet(entt::registry &registry, entt::entity client_entity, const create_entity &packet) {
    auto &client = registry.get<remote_client>(client_entity);
    auto emap_packet = update_entity_map{};

    for (auto &pair : packet.pairs) {
        auto local_entity = registry.create();
        auto remote_entity = pair.entity;
        registry.emplace<entity_owner>(local_entity, client_entity);

        emap_packet.pairs.emplace_back(remote_entity, local_entity);
        client.entity_map.insert(remote_entity, local_entity);
        client.owned_entities.push_back(local_entity);
    }

    client.packet_signal.publish(edyn_packet{std::move(emap_packet)});

    for (auto &pair : packet.pairs) {
        auto remote_entity = pair.entity;
        auto local_entity = client.entity_map.remloc(remote_entity);

        for (auto &comp_ptr : pair.components) {
            assign_component_wrapper(registry, local_entity, *comp_ptr, client.entity_map);
        }
    }
}

static void process_packet(entt::registry &registry, entt::entity client_entity, const update_entity_map &packet) {
    auto &client = registry.get<remote_client>(client_entity);

    for (auto &pair : packet.pairs) {
        auto local_entity = pair.first;
        auto remote_entity = pair.second;
        client.entity_map.insert(remote_entity, local_entity);
    }
}

void server_process_packet(entt::registry &registry, entt::entity client_entity, const edyn_packet &packet) {
    std::visit([&] (auto &&decoded_packet) {
        process_packet(registry, client_entity, decoded_packet);
    }, packet.var);
}

template<typename Component>
void insert_pool_in_snapshot(entt::registry &registry, transient_snapshot &snapshot) {
    auto view = registry.view<Component>();

    if (view.empty()) {
        return;
    }

    auto pool = std::make_unique<pool_snapshot<Component>>();
    pool->component_index = tuple_index_of<Component, unsigned>(networked_components);

    for (auto entity : view) {
        if constexpr(entt::is_eto_eligible_v<Component>) {
            pool->pairs.emplace_back(entity);
        } else {
            auto &comp = view.get(entity);
            pool->pairs.emplace_back(entity, comp);
        }
    }

    snapshot.pools.push_back(std::move(pool));
}

transient_snapshot server_get_transient_snapshot(entt::registry &registry) {
    auto snapshot = transient_snapshot{};

    std::apply([&] (auto ... comp) {
        ((insert_pool_in_snapshot<decltype(comp)>(registry, snapshot)), ...);
    }, transient_components);

    return snapshot;
}

void server_make_client(entt::registry &registry, entt::entity entity) {
    registry.emplace<remote_client>(entity);
}

entt::entity server_make_client(entt::registry &registry) {
    auto entity = registry.create();
    server_make_client(registry, entity);
    return entity;
}

}
