#ifndef EDYN_NETWORKING_PACKET_ENTITY_RESPONSE_HPP
#define EDYN_NETWORKING_PACKET_ENTITY_RESPONSE_HPP

#include <vector>
#include <memory>
#include <entt/entity/fwd.hpp>
#include "edyn/networking/networked_comp.hpp"

namespace edyn {

struct entity_response_component_base {
    size_t component_index;
};

template<typename Component>
struct entity_response_component : public entity_response_component_base {
    Component value;
};

struct entity_response_element {
    entt::entity entity;
    std::vector<std::unique_ptr<entity_response_component_base>> components;
};

struct entity_response {
    std::vector<entity_response_element> elements;
};

template<typename Archive, typename Component>
void serialize(Archive &archive, entity_response_component<Component> &comp) {
    archive(comp.value);
}

namespace detail {
    template<typename Archive, typename... Component>
    void serialize(Archive &archive, entity_response_component_base &comp) {
        archive(comp.component_index);
        size_t i = 0;
        ((i++ == comp.component_index ? serialize(archive, static_cast<entity_response_component<Component> &>(comp)) : void(0)), ...);
    }

    template<typename Archive, typename... Component>
    void serialize(Archive &archive, entity_response_component_base &comp, std::tuple<Component...>) {
        serialize<Archive, Component...>(archive, comp);
    }

    template<typename Func, typename... Component>
    void visit_entity_response_component(entity_response_component_base &comp, Func func, std::tuple<Component...>) {
        size_t i = 0;
        ((i++ == comp.component_index ? func(static_cast<entity_response_component<Component> &>(comp).value) : void(0)), ...);
    }
}

template<typename Archive>
void serialize(Archive &archive, entity_response_component_base &comp) {
    detail::serialize(archive, comp, networked_components);
}

template<typename Archive>
void serialize(Archive &archive, entity_response_element &element) {
    archive(element.entity);
    archive(element.components);
}

template<typename Archive>
void serialize(Archive &archive, entity_response &res) {
    archive(res.elements);
}

template<typename Func>
void visit_entity_response_component(entity_response_component_base &comp, Func func) {
    detail::visit_entity_response_component(comp, func, networked_components);
}

}

#endif // EDYN_NETWORKING_PACKET_ENTITY_RESPONSE_HPP
