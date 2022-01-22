#ifndef EDYN_NETWORKING_NETWORKED_COMPONENT_INDEX_SOURCE_HPP
#define EDYN_NETWORKING_NETWORKED_COMPONENT_INDEX_SOURCE_HPP

#include "edyn/networking/comp/networked_comp.hpp"
#include "edyn/util/tuple_util.hpp"
#include <entt/entity/fwd.hpp>
#include <entt/core/type_info.hpp>

namespace edyn {

struct networked_component_index_source {
    virtual ~networked_component_index_source() = default;

    template<typename Component>
    size_t index_of() const {
        if constexpr(has_type<Component, networked_components_t>::value) {
            return tuple_index_of<Component>(networked_components);
        } else {
            // Get external component index by `entt::type_index`.
            return index_of_id(entt::type_id<Component>().seq());
        }
    }

    template<typename... Component>
    auto indices_of() const {
        return std::array<size_t, sizeof...(Component)>{index_of<Component>()...};
    }

    virtual size_t index_of_id(entt::id_type id) const = 0;

    virtual entt::id_type type_id_of(size_t index) const = 0;
};

template<typename... Component>
struct networked_component_index_source_impl : public networked_component_index_source {
    using components_tuple_t = std::tuple<Component...>;

    networked_component_index_source_impl() = default;
    networked_component_index_source_impl([[maybe_unused]] std::tuple<Component...>) {}

    size_t index_of_id(entt::id_type id) const override {
        auto idx = SIZE_MAX;
        ((entt::type_id<Component>().seq() == id ?
            (idx = edyn::index_of<size_t, Component, components_tuple_t>::value) : (size_t)0), ...);
        EDYN_ASSERT(idx != SIZE_MAX);
        return idx;
    }

    entt::id_type type_id_of(size_t index) const override {
        auto id = entt::id_type{};
        auto tuple = components_tuple_t{};
        visit_tuple(tuple, index, [&] (auto &&c) {
            id = entt::type_id<std::decay_t<decltype(c)>>().seq();
        });
        return id;
    }
};

}

#endif // EDYN_NETWORKING_NETWORKED_COMPONENT_INDEX_SOURCE_HPP
