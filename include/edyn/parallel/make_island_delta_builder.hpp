#ifndef EDYN_PARALLEL_MAKE_ISLAND_DELTA_BUILDER_HPP
#define EDYN_PARALLEL_MAKE_ISLAND_DELTA_BUILDER_HPP

#include <memory>
#include <entt/entity/fwd.hpp>

namespace edyn {

class island_delta_builder;

/**
 * @brief Function type of a factory function that creates instances of a
 * registry delta builder implementation.
 */
using make_island_delta_builder_func_t = std::unique_ptr<island_delta_builder>(*)();

/**
 * @brief Creates a new delta builder.
 *
 * Returns a delta builder implementation that supports handling all shared
 * component types plus any external component set by the user.
 *
 * @return Safe pointer to an instance of a delta builder implementation.
 */
std::unique_ptr<island_delta_builder> make_island_delta_builder(entt::registry &registry);

}

#endif // EDYN_PARALLEL_MAKE_ISLAND_DELTA_BUILDER_HPP
