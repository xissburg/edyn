#ifndef EDYN_NETWORKING_PACKET_ENTITY_COMPONENTS_PAIR_HPP
#define EDYN_NETWORKING_PACKET_ENTITY_COMPONENTS_PAIR_HPP

#include <vector>
#include <memory>
#include <entt/entity/registry.hpp>
#include "edyn/util/tuple_util.hpp"
#include "edyn/networking/packet/util/component_wrapper.hpp"

namespace edyn {

struct entity_components_pair {
    entt::entity entity;
    std::vector<std::unique_ptr<component_wrapper_base>> components;
};

template<typename Archive>
void serialize(Archive &archive, entity_components_pair &pair) {
    archive(pair.entity);
    archive(pair.components);
}

template<typename Component>
void insert_component_in_entity_components_pair(entt::registry &registry, entt::entity entity, entity_components_pair &pair) {
    if (!registry.has<Component>(entity)) {
        return;
    }

    auto index = tuple_index_of<Component, unsigned>(networked_components);

    if constexpr(entt::is_eto_eligible_v<Component>) {
        auto ptr = std::make_unique<component_wrapper<Component>>(component_wrapper<Component>{index});
        pair.components.push_back(std::move(ptr));
    } else {
        auto &comp = registry.get<Component>(entity);
        auto ptr = std::make_unique<component_wrapper<Component>>(component_wrapper<Component>{index, comp});
        pair.components.push_back(std::move(ptr));
    }
}

inline auto make_entity_components_pair(entt::registry &registry, entt::entity entity) {
    auto pair = entity_components_pair{};
    pair.entity = entity;

    std::apply([&] (auto ... comp) {
        ((insert_component_in_entity_components_pair<decltype(comp)>(registry, entity, pair)), ...);
    }, networked_components);

    return pair;
}

}

#endif // EDYN_NETWORKING_PACKET_ENTITY_COMPONENTS_PAIR_HPP
