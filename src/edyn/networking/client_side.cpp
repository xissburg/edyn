#include "edyn/networking/client_side.hpp"
#include "edyn/collision/contact_manifold.hpp"
#include "edyn/comp/tag.hpp"
#include "edyn/networking/packet/entity_request.hpp"
#include "edyn/networking/client_networking_context.hpp"
#include "edyn/parallel/entity_graph.hpp"
#include "edyn/comp/graph_node.hpp"
#include "edyn/comp/graph_edge.hpp"
#include "edyn/comp/dirty.hpp"
#include "edyn/parallel/merge/merge_component.hpp"
#include "edyn/parallel/merge/merge_contact_point.hpp"
#include "edyn/parallel/merge/merge_contact_manifold.hpp"
#include "edyn/parallel/merge/merge_constraint.hpp"
#include "edyn/parallel/merge/merge_tree_view.hpp"
#include "edyn/parallel/merge/merge_collision_exclusion.hpp"
#include "edyn/edyn.hpp"
#include <entt/entity/registry.hpp>

namespace edyn {

static void process_packet(entt::registry &registry, const entity_request &req) {

}

template<typename Component>
static void emplace_component(entt::registry &registry, entt::entity entity,
                              const entity_response_component_base &res_comp,
                              const edyn::entity_map &entity_map) {
    auto &typed_res_comp = static_cast<const entity_response_component<Component> &>(res_comp);
    auto comp = typed_res_comp.value;
    auto ctx = merge_context{&registry, &entity_map};
    merge<Component>(nullptr, comp, ctx);
    registry.emplace<Component>(entity, comp);
}

template<>
void emplace_component<rigidbody_tag>(entt::registry &registry, entt::entity entity,
                                      const entity_response_component_base &comp,
                                      const edyn::entity_map &entity_map) {
    registry.emplace<rigidbody_tag>(entity);

    auto non_connecting = !registry.has<dynamic_tag>(entity);
    auto node_index = registry.ctx<entity_graph>().insert_node(entity, non_connecting);
    registry.emplace<graph_node>(entity, node_index);
}

template<>
void emplace_component<contact_manifold>(entt::registry &registry, entt::entity entity,
                                         const entity_response_component_base &comp,
                                         const edyn::entity_map &entity_map) {
    auto &typed_comp = static_cast<const entity_response_component<contact_manifold> &>(comp);
    auto manifold = typed_comp.value;
    auto ctx = merge_context{&registry, &entity_map};
    merge<contact_manifold>(nullptr, manifold, ctx);
    registry.emplace<contact_manifold>(entity, manifold);

    auto node_index0 = registry.get<graph_node>(manifold.body[0]).node_index;
    auto node_index1 = registry.get<graph_node>(manifold.body[1]).node_index;
    auto edge_index = registry.ctx<entity_graph>().insert_edge(entity, node_index0, node_index1);
    registry.emplace<graph_edge>(entity, edge_index);
}

template<typename... Component>
static void assign_component(entt::registry &registry, entt::entity entity,
                             const entity_response_component_base &comp,
                             const edyn::entity_map &entity_map, std::tuple<Component...>) {
    size_t i = 0;
    ((i++ == comp.component_index ? emplace_component<Component>(registry, entity, comp, entity_map) : void(0)), ...);
}

static void process_packet(entt::registry &registry, const entity_response &res) {
    auto &ctx = registry.ctx<client_networking_context>();

    for (auto &element : res.elements) {
        if (ctx.entity_map.has_rem((element.entity))) {
            continue;
        }

        auto local_entity = registry.create();
        ctx.entity_map.insert(element.entity, local_entity);

        for (auto &comp_ptr : element.components) {
            assign_component(registry, local_entity, *comp_ptr, ctx.entity_map, networked_components);
        }
    }
}

template<typename Component>
void import_pool(entt::registry &registry, const pool_snapshot<Component> &pool) {
    if constexpr(entt::is_eto_eligible_v<Component>) {
        return;
    }

    auto &ctx = registry.ctx<client_networking_context>();

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
