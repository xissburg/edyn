#ifndef EDYN_COLLIDE_SHOULD_COLLIDE_HPP
#define EDYN_COLLIDE_SHOULD_COLLIDE_HPP

#include <entt/entity/fwd.hpp>

namespace edyn {

bool should_collide_default(const entt::registry &, entt::entity, entt::entity);
using should_collide_func_t = decltype(&should_collide_default);

/**
 * @brief Overrides the default collision filtering function, which checks
 * collision groups and masks. Remember to return false if both entities
 * are the same.
 * @param registry Data source.
 * @param func The function.
 */
void set_should_collide(entt::registry &registry, should_collide_func_t func);

}

#endif // EDYN_COLLIDE_SHOULD_COLLIDE_HPP
