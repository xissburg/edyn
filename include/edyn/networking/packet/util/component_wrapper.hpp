#ifndef EDYN_NETWORKING_PACKET_UTIL_COMPONENT_WRAPPER_HPP
#define EDYN_NETWORKING_PACKET_UTIL_COMPONENT_WRAPPER_HPP

#include "edyn/collision/contact_manifold.hpp"
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
#include <type_traits>

namespace edyn {

struct component_wrapper_base {
    unsigned component_index;
};

template<typename Component>
struct component_wrapper : public component_wrapper_base {
    Component value;
};

namespace detail {
    template<typename Component, typename Archive>
    void serialize_derived(Archive &archive, std::shared_ptr<component_wrapper_base> &comp) {
        if (!comp) {
            comp.reset(new component_wrapper<Component>);
        }

        archive(std::static_pointer_cast<component_wrapper<Component>>(comp)->value);
    }

    template<typename Archive, typename... Component>
    void serialize(Archive &archive, std::shared_ptr<component_wrapper_base> &comp) {
        if constexpr(Archive::is_output::value) {
            archive(comp->component_index);
            unsigned i = 0;
            ((i++ == comp->component_index ? serialize_derived<Component>(archive, comp) : void(0)), ...);
        } else {
            unsigned component_index;
            archive(component_index);

            unsigned i = 0;
            ((i++ == component_index ? serialize_derived<Component>(archive, comp) : void(0)), ...);
            comp->component_index = component_index;
        }
    }

    template<typename Archive, typename... Component>
    void serialize(Archive &archive, std::shared_ptr<component_wrapper_base> &comp, std::tuple<Component...>) {
        serialize<Archive, Component...>(archive, comp);
    }

    template<typename Func, typename... Component>
    void visit_component_wrapper(component_wrapper_base &comp, Func func, std::tuple<Component...>) {
        unsigned i = 0;
        ((i++ == comp.component_index ? func(static_cast<component_wrapper<Component> &>(comp).value) : void(0)), ...);
    }
}

template<typename Archive>
void serialize(Archive &archive, std::shared_ptr<component_wrapper_base> &comp) {
    detail::serialize(archive, comp, networked_components);
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

#endif // EDYN_NETWORKING_PACKET_UTIL_COMPONENT_WRAPPER_HPP
