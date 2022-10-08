#ifndef EDYN_UTIL_GRAVITY_UTIL_HPP
#define EDYN_UTIL_GRAVITY_UTIL_HPP

#include "edyn/math/vector3.hpp"
#include <entt/entity/fwd.hpp>

namespace edyn {

/**
 * @brief Get default gravity.
 * This value is assigned as the gravitational acceleration for all new rigid bodies.
 * @param registry Data source.
 * @return Gravity acceleration vector.
 */
vector3 get_gravity(const entt::registry &registry);

/**
 * @brief Changes the default gravity acceleration.
 * This value is assigned as the gravitational acceleration for all new rigid bodies.
 * @param registry Data source.
 * @param gravity The new default gravity acceleration.
 */
void set_gravity(entt::registry &registry, vector3 gravity);

}

#endif // EDYN_UTIL_GRAVITY_UTIL_HPP
