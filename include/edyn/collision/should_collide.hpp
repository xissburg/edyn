#ifndef EDYN_COLLIDE_SHOULD_COLLIDE_HPP
#define EDYN_COLLIDE_SHOULD_COLLIDE_HPP

#include <entt/entity/fwd.hpp>

namespace edyn {

bool should_collide_default(entt::registry &, entt::entity, entt::entity);

}

#endif // EDYN_COLLIDE_SHOULD_COLLIDE_HPP
