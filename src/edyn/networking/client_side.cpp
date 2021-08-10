#include "edyn/networking/client_side.hpp"
#include "edyn/collision/contact_manifold.hpp"
#include "edyn/comp/tag.hpp"
#include "edyn/constraints/soft_distance_constraint.hpp"
#include "edyn/networking/packet/entity_request.hpp"
#include "edyn/networking/client_networking_context.hpp"
#include "edyn/comp/dirty.hpp"
#include "edyn/parallel/entity_graph.hpp"
#include "edyn/edyn.hpp"
#include <entt/entity/registry.hpp>

namespace edyn {

void on_construct_networked_entity(entt::registry &registry, entt::entity entity) {
    auto &ctx = registry.ctx<client_networking_context>();

    if (!ctx.importing_entities) {
        ctx.created_entities.push_back(entity);
    }
}

void init_networking_client(entt::registry &registry) {
    registry.set<client_networking_context>();
    registry.on_construct<rigidbody_tag>().connect<&on_construct_networked_entity>();
    registry.on_construct<soft_distance_constraint>().connect<&on_construct_networked_entity>();
}

void update_networking_client(entt::registry &registry) {
    auto &ctx = registry.ctx<client_networking_context>();

    if (!ctx.created_entities.empty()) {
        create_entity packet;

        for (auto entity : ctx.created_entities) {
            auto pair = make_entity_components_pair(registry, entity);
            packet.pairs.push_back(std::move(pair));
        }

        ctx.packet_signal.publish(edyn_packet{std::move(packet)});
        ctx.created_entities.clear();
    }
}

static void process_packet(entt::registry &registry, const update_entity_map &emap) {
    auto &ctx = registry.ctx<client_networking_context>();

    for (auto &pair : emap.pairs) {
        auto local_entity = pair.first;
        auto remote_entity = pair.second;
        ctx.entity_map.insert(remote_entity, local_entity);
    }
}

static void process_packet(entt::registry &registry, const entity_request &req) {

}

static void process_packet(entt::registry &registry, const entity_response &res) {
    auto &ctx = registry.ctx<client_networking_context>();
    ctx.importing_entities = true;

    auto emap_packet = update_entity_map{};

    for (auto &pair : res.pairs) {
        auto remote_entity = pair.entity;

        if (ctx.entity_map.has_rem(remote_entity)) {
            continue;
        }

        auto local_entity = registry.create();
        ctx.entity_map.insert(remote_entity, local_entity);
        emap_packet.pairs.emplace_back(remote_entity, local_entity);

        for (auto &comp_ptr : pair.components) {
            assign_component_wrapper(registry, local_entity, *comp_ptr, ctx.entity_map);
        }
    }

    ctx.importing_entities = false;

    if (!emap_packet.pairs.empty()) {
        ctx.packet_signal.publish(edyn_packet{std::move(emap_packet)});
    }
}

static void process_packet(entt::registry &registry, const create_entity &packet) {
    auto &ctx = registry.ctx<client_networking_context>();
    ctx.importing_entities = true;

    auto emap_packet = update_entity_map{};

    for (auto &pair : packet.pairs) {
        auto remote_entity = pair.entity;
        auto local_entity = registry.create();
        ctx.entity_map.insert(remote_entity, local_entity);
        emap_packet.pairs.emplace_back(remote_entity, local_entity);

        for (auto &comp_ptr : pair.components) {
            assign_component_wrapper(registry, local_entity, *comp_ptr, ctx.entity_map);
        }
    }

    ctx.importing_entities = false;

    if (!emap_packet.pairs.empty()) {
        ctx.packet_signal.publish(edyn_packet{std::move(emap_packet)});
    }
}

template<typename Component>
void import_pool(entt::registry &registry, const pool_snapshot<Component> &pool) {
    if constexpr(entt::is_eto_eligible_v<Component>) {
        return;
    }

    auto &ctx = registry.ctx<client_networking_context>();
    ctx.importing_entities = true;

    for (auto &pair : pool.pairs) {
        auto remote_entity = pair.first;

        if (ctx.entity_map.has_rem(pair.first)) {
            auto local_entity = ctx.entity_map.remloc(remote_entity);

            if (registry.valid(local_entity)) {
                if (registry.has<Component>(local_entity)) {
                    registry.replace<Component>(local_entity, pair.second);
                    edyn::refresh<Component>(registry, local_entity);
                } else {
                    registry.emplace<Component>(local_entity, pair.second);
                    registry.emplace_or_replace<dirty>(local_entity).template created<Component>();
                }
            } else {
                ctx.entity_map.erase_loc(local_entity);
                ctx.request_entity_signal.publish(remote_entity);
            }
        } else {
            ctx.request_entity_signal.publish(remote_entity);
        }
    }

    ctx.importing_entities = false;
}

void process_pool(entt::registry &registry, const pool_snapshot_base &pool) {
    std::apply([&] (auto ... comp) {
        size_t i = 0;
        ((i++ == pool.component_index ? import_pool(registry, static_cast<const pool_snapshot<decltype(comp)> &>(pool)) : void(0)), ...);
    }, networked_components);
}

static void process_packet(entt::registry &registry, const transient_snapshot &snapshot) {
    for (auto &pool_ptr : snapshot.pools) {
        process_pool(registry, *pool_ptr);
    }
}

void client_process_packet(entt::registry &registry, const edyn_packet &packet) {
    std::visit([&] (auto &&inner_packet) {
        process_packet(registry, inner_packet);
    }, packet.var);
}

}
