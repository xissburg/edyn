#include "edyn/networking/client_side.hpp"
#include "edyn/collision/contact_manifold.hpp"
#include "edyn/config/config.h"
#include "edyn/edyn.hpp"
#include "edyn/networking/entity_owner.hpp"
#include "edyn/parallel/entity_graph.hpp"
#include "edyn/comp/graph_edge.hpp"
#include "edyn/comp/graph_node.hpp"
#include "edyn/constraints/soft_distance_constraint.hpp"
#include "edyn/networking/networked_comp.hpp"
#include "edyn/networking/packet/entity_request.hpp"
#include "edyn/networking/packet/util/pool_snapshot.hpp"
#include "edyn/networking/client_networking_context.hpp"
#include "edyn/comp/dirty.hpp"
#include "edyn/comp/tag.hpp"
#include "edyn/parallel/merge/merge_component.hpp"
#include "edyn/networking/extrapolation_job.hpp"
#include "edyn/networking/client_import_pool.hpp"
#include "edyn/time/time.hpp"
#include "edyn/util/tuple_util.hpp"
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

        if (ctx.entity_map.has_loc(entity)) {
            ctx.entity_map.erase_loc(entity);
        }
    }
}

void on_construct_entity_owner(entt::registry &registry, entt::entity entity) {
    auto &ctx = registry.ctx<client_networking_context>();
    auto &owner = registry.get<entity_owner>(entity);

    if (owner.client_entity == ctx.client_entity) {
        ctx.owned_entities.emplace(entity);
    }
}

void on_destroy_entity_owner(entt::registry &registry, entt::entity entity) {
    auto &ctx = registry.ctx<client_networking_context>();
    auto &owner = registry.get<entity_owner>(entity);

    if (owner.client_entity == ctx.client_entity) {
        ctx.owned_entities.erase(entity);
    }
}

void init_networking_client(entt::registry &registry) {
    registry.set<client_networking_context>();

    registry.on_construct<networked_tag>().connect<&on_construct_networked_entity>();
    registry.on_destroy<networked_tag>().connect<&on_destroy_networked_entity>();
    registry.on_construct<entity_owner>().connect<&on_construct_entity_owner>();
    registry.on_destroy<entity_owner>().connect<&on_destroy_entity_owner>();
}

void deinit_networking_client(entt::registry &registry) {
    registry.unset<client_networking_context>();

    registry.on_construct<networked_tag>().disconnect<&on_construct_networked_entity>();
    registry.on_destroy<networked_tag>().disconnect<&on_destroy_networked_entity>();
    registry.on_construct<entity_owner>().disconnect<&on_construct_entity_owner>();
    registry.on_destroy<entity_owner>().disconnect<&on_destroy_entity_owner>();
}

void update_networking_client(entt::registry &registry) {
    auto &ctx = registry.ctx<client_networking_context>();

    if (!ctx.created_entities.empty()) {
        packet::create_entity packet;
        packet.entities = ctx.created_entities;

        for (auto entity : ctx.created_entities) {
            (*ctx.insert_entity_components_func)(registry, entity, packet.pools);
        }

        std::sort(packet.pools.begin(), packet.pools.end(), [] (auto &&lhs, auto &&rhs) {
            return lhs.component_index < rhs.component_index;
        });

        ctx.packet_signal.publish(packet::edyn_packet{std::move(packet)});
        ctx.created_entities.clear();
    }

    if (!ctx.destroyed_entities.empty()) {
        packet::destroy_entity packet;
        packet.entities = std::move(ctx.destroyed_entities);
        ctx.packet_signal.publish(packet::edyn_packet{std::move(packet)});
    }

    auto time = performance_time();

    if (time - ctx.last_snapshot_time > 1 / ctx.snapshot_rate) {
        auto packet = packet::transient_snapshot{};

        for (auto entity : ctx.owned_entities) {
            EDYN_ASSERT(registry.all_of<networked_tag>(entity));
            (*ctx.insert_transient_components_func)(registry, entity, packet.pools);
        }

        ctx.last_snapshot_time = time;

        if (!packet.pools.empty()) {
            ctx.packet_signal.publish(packet::edyn_packet{std::move(packet)});
        }
    }
}

static void process_packet(entt::registry &registry, const packet::client_created &packet) {
    auto &ctx = registry.ctx<client_networking_context>();
    ctx.importing_entities = true;

    auto remote_entity = packet.client_entity;
    auto local_entity = registry.create();
    edyn::tag_external_entity(registry, local_entity, false);

    EDYN_ASSERT(ctx.client_entity == entt::null);
    ctx.client_entity = local_entity;
    ctx.client_entity_assigned_signal.publish();
    ctx.entity_map.insert(remote_entity, local_entity);

    auto emap_packet = packet::update_entity_map{};
    emap_packet.pairs.emplace_back(remote_entity, local_entity);
    ctx.packet_signal.publish(packet::edyn_packet{std::move(emap_packet)});

    ctx.importing_entities = false;
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

    for (auto remote_entity : res.entities) {

        if (ctx.entity_map.has_rem(remote_entity)) {
            continue;
        }

        auto local_entity = registry.create();
        ctx.entity_map.insert(remote_entity, local_entity);
        emap_packet.pairs.emplace_back(remote_entity, local_entity);
    }

    for (auto &pool : res.pools) {
        (*ctx.import_pool_func)(registry, pool);
    }

    for (auto remote_entity : res.entities) {
        auto local_entity = ctx.entity_map.remloc(remote_entity);
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

    // Collect new entity mappings to send back to server.
    auto emap_packet = packet::update_entity_map{};

    // Create entities first...
    for (auto remote_entity : packet.entities) {
        if (ctx.entity_map.has_rem(remote_entity)) continue;
        auto local_entity = registry.create();
        ctx.entity_map.insert(remote_entity, local_entity);
        emap_packet.pairs.emplace_back(remote_entity, local_entity);
    }

    if (!emap_packet.pairs.empty()) {
        ctx.packet_signal.publish(packet::edyn_packet{std::move(emap_packet)});
    }

    // ... assign components later so that entity references will be available
    // to be mapped into the local registry.
    for (auto &pool : packet.pools) {
        (*ctx.import_pool_func)(registry, pool);
    }

    for (auto remote_entity : packet.entities) {
        auto local_entity = ctx.entity_map.remloc(remote_entity);
        registry.emplace_or_replace<networked_tag>(local_entity);
    }

    // Create nodes and edges in entity graph.
    for (auto remote_entity : packet.entities) {
        auto local_entity = ctx.entity_map.remloc(remote_entity);

        if (registry.any_of<rigidbody_tag, external_tag>(local_entity) &&
            !registry.all_of<graph_node>(local_entity)) {
            auto non_connecting = !registry.any_of<procedural_tag>(local_entity);
            auto node_index = registry.ctx<entity_graph>().insert_node(local_entity, non_connecting);
            registry.emplace<graph_node>(local_entity, node_index);
        }
    }

    for (auto remote_entity : packet.entities) {
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
}

static void process_packet(entt::registry &registry, const packet::destroy_entity &packet) {
    auto &ctx = registry.ctx<client_networking_context>();
    ctx.importing_entities = true;

    for (auto remote_entity : packet.entities) {
        if (!ctx.entity_map.has_rem(remote_entity)) continue;

        auto local_entity = ctx.entity_map.remloc(remote_entity);
        ctx.entity_map.erase_rem(remote_entity);

        if (registry.valid(local_entity)) {
            registry.destroy(local_entity);
        }
    }

    ctx.importing_entities = false;
}

static void process_packet(entt::registry &registry, const packet::transient_snapshot &snapshot) {
    auto &ctx = registry.ctx<client_networking_context>();

    for (auto &pool : snapshot.pools) {
        (*ctx.import_pool_func)(registry, pool);
    }
}

static void process_packet(entt::registry &registry, const packet::general_snapshot &snapshot) {
    auto &ctx = registry.ctx<client_networking_context>();

    for (auto &pool : snapshot.pools) {
        (*ctx.import_pool_func)(registry, pool);
    }
}

void client_process_packet(entt::registry &registry, const packet::edyn_packet &packet) {
    std::visit([&] (auto &&inner_packet) {
        process_packet(registry, inner_packet);
    }, packet.var);
}

entt::sink<void()> on_client_entity_assigned(entt::registry &registry) {
    auto &ctx = registry.ctx<client_networking_context>();
    return entt::sink{ctx.client_entity_assigned_signal};
}

bool client_owns_entity(entt::registry &registry, entt::entity entity) {
    auto &ctx = registry.ctx<client_networking_context>();
    return ctx.client_entity == registry.get<entity_owner>(entity).client_entity;
}

}
