#ifndef EDYN_NETWORKING_UTIL_IS_FULLY_OWNED_BY_CLIENT_HPP
#define EDYN_NETWORKING_UTIL_IS_FULLY_OWNED_BY_CLIENT_HPP

#include <entt/entity/fwd.hpp>

namespace edyn {

bool is_fully_owned_by_client(const entt::registry &registry, entt::entity client_entity, entt::entity entity);

}

#endif // EDYN_NETWORKING_UTIL_IS_FULLY_OWNED_BY_CLIENT_HPP
