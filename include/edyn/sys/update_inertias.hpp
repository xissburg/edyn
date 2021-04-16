#ifndef EDYN_SYS_UPDATE_INERTIA_HPP
#define EDYN_SYS_UPDATE_INERTIA_HPP

#include <entt/fwd.hpp>

namespace edyn {

/**
 * @brief Updates all `inertia_world_inv` by rotating their inertia tensor
 * to world space.
 */
void update_inertias(entt::registry &);
    
}

#endif // EDYN_SYS_UPDATE_INERTIA_HPP
