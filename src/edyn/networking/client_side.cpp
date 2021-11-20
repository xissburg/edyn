#include "edyn/networking/client_side.hpp"
#include "edyn/collision/contact_manifold.hpp"
#include "edyn/comp/tag.hpp"
#include "edyn/constraints/soft_distance_constraint.hpp"
#include "edyn/networking/packet/entity_request.hpp"
#include "edyn/networking/packet/util/pool_snapshot.hpp"
#include "edyn/networking/client_networking_context.hpp"
#include "edyn/comp/dirty.hpp"
#include "edyn/parallel/entity_graph.hpp"
#include "edyn/networking/extrapolation_job.hpp"
#include "edyn/edyn.hpp"
#include <entt/entity/registry.hpp>

namespace edyn {

void on_construct_networked_entity(entt::registry &registry, entt::entity entity) {
    auto &ctx = registry.ctx<client_networking_context>();

    if (!ctx.importing_entities) {
        ctx.created_entities.push_back(entity);
    }
}

void on_destroy_networked_entity(entt::registry &registry, entt::entity entity) {
    auto &ctx = registry.ctx<client_networking_context>();

    if (!ctx.importing_entities) {
        ctx.destroyed_entities.push_back(entity);
    }
}

void init_networking_client(entt::registry &registry) {
    registry.set<client_networking_context>();

    registry.on_construct<networked_tag>().connect<&on_construct_networked_entity>();
    registry.on_destroy<networked_tag>().connect<&on_destroy_networked_entity>();
}

void update_networking_client(entt::registry &registry) {
    auto &ctx = registry.ctx<client_networking_context>();

    if (!ctx.created_entities.empty()) {
        packet::create_entity packet;

        for (auto entity : ctx.created_entities) {
            auto pair = make_entity_components_pair(registry, entity);
            packet.pairs.push_back(std::move(pair));
        }

        ctx.packet_signal.publish(packet::edyn_packet{std::move(packet)});
        ctx.created_entities.clear();
    }

    if (!ctx.destroyed_entities.empty()) {
        packet::destroy_entity packet;
        packet.entities = std::move(ctx.destroyed_entities);
        ctx.packet_signal.publish(packet::edyn_packet{std::move(packet)});
    }
}

static void process_packet(entt::registry &registry, const packet::update_entity_map &emap) {
    auto &ctx = registry.ctx<client_networking_context>();

    for (auto &pair : emap.pairs) {
        auto local_entity = pair.first;
        auto remote_entity = pair.second;
        ctx.entity_map.insert(remote_entity, local_entity);
    }
}

static void process_packet(entt::registry &registry, const packet::entity_request &req) {

}

static void process_packet(entt::registry &registry, const packet::entity_response &res) {
    auto &ctx = registry.ctx<client_networking_context>();
    ctx.importing_entities = true;

    auto emap_packet = packet::update_entity_map{};

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

        registry.emplace<networked_tag>(local_entity);
    }

    ctx.importing_entities = false;

    if (!emap_packet.pairs.empty()) {
        ctx.packet_signal.publish(packet::edyn_packet{std::move(emap_packet)});
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

static void process_packet(entt::registry &registry, const packet::create_entity &packet) {
    auto &ctx = registry.ctx<client_networking_context>();
    ctx.importing_entities = true;

    auto emap_packet = packet::update_entity_map{};

    // Create entities first...
    for (auto &pair : packet.pairs) {
        auto remote_entity = pair.entity;
        auto local_entity = registry.create();
        ctx.entity_map.insert(remote_entity, local_entity);
        emap_packet.pairs.emplace_back(remote_entity, local_entity);
    }

    // ... assign components later so that entity references will be available
    // to be mapped into the local registry.
    for (auto &pair : packet.pairs) {
        auto remote_entity = pair.entity;
        auto local_entity = ctx.entity_map.remloc(remote_entity);

        for (auto &comp_ptr : pair.components) {
            assign_component_wrapper(registry, local_entity, *comp_ptr, ctx.entity_map);
        }

        registry.emplace<networked_tag>(local_entity);
    }

    // Create nodes and edges in entity graph.
    for (auto &pair : packet.pairs) {
        auto remote_entity = pair.entity;
        auto local_entity = ctx.entity_map.remloc(remote_entity);

        if (registry.any_of<rigidbody_tag, external_tag>(local_entity)) {
            auto non_connecting = !registry.any_of<procedural_tag>(local_entity);
            auto node_index = registry.ctx<entity_graph>().insert_node(local_entity, non_connecting);
            registry.emplace<graph_node>(local_entity, node_index);
        }
    }

    for (auto &pair : packet.pairs) {
        auto remote_entity = pair.entity;
        auto local_entity = ctx.entity_map.remloc(remote_entity);

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

    ctx.importing_entities = false;

    if (!emap_packet.pairs.empty()) {
        ctx.packet_signal.publish(packet::edyn_packet{std::move(emap_packet)});
    }
}

static void process_packet(entt::registry &registry, const packet::destroy_entity &packet) {
    auto &ctx = registry.ctx<client_networking_context>();
    ctx.importing_entities = true;

    for (auto remote_entity : packet.entities) {
        if (ctx.entity_map.has_rem(remote_entity)) {
            auto local_entity = ctx.entity_map.remloc(remote_entity);
            registry.destroy(local_entity);
        }
    }

    ctx.importing_entities = false;
}

template<typename Component>
void import_pool(entt::registry &registry, const std::vector<std::pair<entt::entity, Component>> &pool) {
    if constexpr(std::is_empty_v<Component>) {
        return;
    }

    auto &ctx = registry.ctx<client_networking_context>();
    ctx.importing_entities = true;

    for (auto &pair : pool) {
        auto remote_entity = pair.first;

        if (ctx.entity_map.has_rem(pair.first)) {
            auto local_entity = ctx.entity_map.remloc(remote_entity);

            if (registry.valid(local_entity)) {
                if (registry.any_of<Component>(local_entity)) {
                    registry.replace<Component>(local_entity, pair.second);
                    edyn::refresh<Component>(registry, local_entity);
                } else {
                    registry.emplace<Component>(local_entity, pair.second);
                    registry.emplace_or_replace<dirty>(local_entity).template created<Component>();
                }
            }
        } else {
            ctx.request_entity_signal.publish(remote_entity);
        }
    }

    ctx.importing_entities = false;
}

static void process_packet(entt::registry &registry, const packet::transient_snapshot &snapshot) {
    import_pool(registry, snapshot.positions);
    import_pool(registry, snapshot.orientations);
    import_pool(registry, snapshot.linvels);
    import_pool(registry, snapshot.angvels);

    //auto job = new extrapolation_job();
}

void client_process_packet(entt::registry &registry, const packet::edyn_packet &packet) {
    std::visit([&] (auto &&inner_packet) {
        process_packet(registry, inner_packet);
    }, packet.var);
}

}
