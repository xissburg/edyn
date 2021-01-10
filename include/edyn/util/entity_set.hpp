#ifndef EDYN_UTIL_ENTITY_COLLECTION_HPP
#define EDYN_UTIL_ENTITY_COLLECTION_HPP

#include <entt/fwd.hpp>
#include <unordered_set>

namespace edyn {

using entity_set = std::unordered_set<entt::entity>;

}

#endif // EDYN_UTIL_ENTITY_COLLECTION_HPP