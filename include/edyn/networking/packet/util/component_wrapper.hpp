#ifndef EDYN_NETWORKING_PACKET_COMPONENT_WRAPPER_HPP
#define EDYN_NETWORKING_PACKET_COMPONENT_WRAPPER_HPP

#include "edyn/constraints/soft_distance_constraint.hpp"
#include "edyn/networking/networked_comp.hpp"
#include "edyn/comp/graph_node.hpp"
#include "edyn/comp/graph_edge.hpp"
#include "edyn/parallel/merge/merge_component.hpp"
#include "edyn/parallel/merge/merge_contact_point.hpp"
#include "edyn/parallel/merge/merge_contact_manifold.hpp"
#include "edyn/parallel/merge/merge_constraint.hpp"
#include "edyn/parallel/merge/merge_tree_view.hpp"
#include "edyn/parallel/merge/merge_collision_exclusion.hpp"

namespace edyn {

struct component_wrapper_base {
    unsigned component_index;
};

template<typename Component>
struct component_wrapper : public component_wrapper_base {
    Component value;
};

namespace detail {
    template<typename Archive, typename... Component>
    void serialize(Archive &archive, component_wrapper_base &comp) {
        archive(comp.component_index);
        unsigned i = 0;
        ((i++ == comp.component_index ? serialize(archive, static_cast<component_wrapper<Component> &>(comp)) : void(0)), ...);
    }

    template<typename Archive, typename... Component>
    void serialize(Archive &archive, component_wrapper_base &comp, std::tuple<Component...>) {
        serialize<Archive, Component...>(archive, comp);
    }

    template<typename Func, typename... Component>
    void visit_component_wrapper(component_wrapper_base &comp, Func func, std::tuple<Component...>) {
        unsigned i = 0;
        ((i++ == comp.component_index ? func(static_cast<component_wrapper<Component> &>(comp).value) : void(0)), ...);
    }
}

template<typename Archive>
void serialize(Archive &archive, component_wrapper_base &comp) {
    detail::serialize(archive, comp, networked_components);
}

template<typename Archive, typename Component>
void serialize(Archive &archive, component_wrapper<Component> &comp) {
    archive(comp.value);
}

template<typename Func>
void visit_component_wrapper(component_wrapper_base &comp, Func func) {
    detail::visit_component_wrapper(comp, func, networked_components);
}

template<typename Component>
void emplace_component_wrapper(entt::registry &registry, entt::entity entity,
                               const component_wrapper_base &wrapper,
                               const edyn::entity_map &entity_map) {
    auto &typed_wrapper = static_cast<const component_wrapper<Component> &>(wrapper);
    auto comp = typed_wrapper.value;
    auto ctx = merge_context{&registry, &entity_map};
    merge<Component>(nullptr, comp, ctx);
    registry.emplace<Component>(entity, comp);
}

template<>
inline void emplace_component_wrapper<rigidbody_tag>(entt::registry &registry, entt::entity entity,
                                              const component_wrapper_base &comp,
                                              const edyn::entity_map &entity_map) {
    registry.emplace<rigidbody_tag>(entity);

    auto non_connecting = !registry.any_of<dynamic_tag>(entity);
    auto node_index = registry.ctx<entity_graph>().insert_node(entity, non_connecting);
    registry.emplace<graph_node>(entity, node_index);
}

template<typename Constraint>
void emplace_constraint_wrapper(entt::registry &registry, entt::entity entity,
                                const component_wrapper_base &wrapper,
                                const edyn::entity_map &entity_map) {
    auto &typed_wrapper = static_cast<const component_wrapper<Constraint> &>(wrapper);
    auto constraint = typed_wrapper.value;
    auto ctx = merge_context{&registry, &entity_map};
    merge<Constraint>(nullptr, constraint, ctx);
    registry.emplace<Constraint>(entity, constraint);

    auto node_index0 = registry.get<graph_node>(constraint.body[0]).node_index;
    auto node_index1 = registry.get<graph_node>(constraint.body[1]).node_index;
    auto edge_index = registry.ctx<entity_graph>().insert_edge(entity, node_index0, node_index1);
    registry.emplace<graph_edge>(entity, edge_index);
}

template<>
inline void emplace_component_wrapper<contact_manifold>(entt::registry &registry, entt::entity entity,
                                                 const component_wrapper_base &wrapper,
                                                 const edyn::entity_map &entity_map) {
    emplace_constraint_wrapper<contact_manifold>(registry, entity, wrapper, entity_map);
}

template<>
inline void emplace_component_wrapper<soft_distance_constraint>(entt::registry &registry, entt::entity entity,
                                                 const component_wrapper_base &wrapper,
                                                 const edyn::entity_map &entity_map) {
    emplace_constraint_wrapper<soft_distance_constraint>(registry, entity, wrapper, entity_map);
}

namespace detail {
    template<typename... Component>
    void assign_component_wrapper(entt::registry &registry, entt::entity entity,
                                const component_wrapper_base &wrapper,
                                const edyn::entity_map &entity_map, std::tuple<Component...>) {
        size_t i = 0;
        ((i++ == wrapper.component_index ? edyn::emplace_component_wrapper<Component>(registry, entity, wrapper, entity_map) : void(0)), ...);
    }
}

template<typename... Component>
void assign_component_wrapper(entt::registry &registry, entt::entity entity,
                              const component_wrapper_base &wrapper,
                              const edyn::entity_map &entity_map) {
    detail::assign_component_wrapper(registry, entity, wrapper, entity_map, networked_components);
}

}

#endif // EDYN_NETWORKING_PACKET_COMPONENT_WRAPPER_HPP
