#ifndef EDYN_UTIL_GET_COMPONENT_INDEX_HPP
#define EDYN_UTIL_GET_COMPONENT_INDEX_HPP

#include "edyn/context/settings.hpp"
#include "edyn/replication/component_index_source.hpp"
#include <entt/entity/registry.hpp>

namespace edyn {

/**
 * @brief Get index of a component type among all shared components within the
 * library. Also supports any registered external component.
 * @tparam Component The component type.
 * @param registry Data source.
 * @return Component index.
 */
template<typename Component>
size_t get_component_index(entt::registry &registry) {
    auto &settings = registry.ctx().at<edyn::settings>();
    return settings.index_source->index_of<Component>();
}

/**
 * @brief Get indices of a sequence of component types among all shared
 * components in the library. Also supports any registered external component.
 * @tparam IndexType Desired integral index type.
 * @tparam Component The component type.
 * @param registry Data source.
 * @return Component index.
 */
template<typename IndexType, typename... Component>
auto get_component_indices(entt::registry &registry) {
    auto &settings = registry.ctx().at<edyn::settings>();
    return settings.index_source->indices_of<IndexType, Component...>();
}

}

#endif // EDYN_UTIL_GET_COMPONENT_INDEX_HPP
