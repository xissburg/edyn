#ifndef EDYN_PARALLEL_COMPONENT_INDEX_SOURCE_HPP
#define EDYN_PARALLEL_COMPONENT_INDEX_SOURCE_HPP

#include "edyn/comp/shared_comp.hpp"
#include "edyn/util/tuple_util.hpp"
#include <entt/entity/fwd.hpp>
#include <entt/core/type_info.hpp>

namespace edyn {

/**
 * Provides a means to obtain a stable index for each component type, both
 * standard and external. The indices correspond to the location of the
 * component in the tuple of components.
 */
struct component_index_source {
    virtual ~component_index_source() = default;

    template<typename Component>
    size_t index_of() const {
        if constexpr(has_type<Component, shared_components_t>::value) {
            return tuple_index_of<Component>(shared_components);
        } else {
            // Get external component index by `entt::type_index`.
            return index_of_id(entt::type_id<Component>().seq());
        }
    }

    virtual size_t index_of_id(entt::id_type id) const = 0;
};

template<typename... Component>
struct external_component_index_source : public component_index_source {
    using components_tuple_t = std::tuple<Component...>;

    size_t index_of_id(entt::id_type id) const override {
        auto idx = SIZE_MAX;
        ((entt::type_id<Component>().seq() == id ?
            (idx = edyn::index_of<size_t, Component, components_tuple_t>::value) : (size_t)0), ...);
        EDYN_ASSERT(idx != SIZE_MAX);
        return idx;
    }
};

}

#endif // EDYN_PARALLEL_COMPONENT_INDEX_SOURCE_HPP
