#ifndef EDYN_UTIL_ENTITY_COLLECTION_HPP
#define EDYN_UTIL_ENTITY_COLLECTION_HPP

#include <unordered_set>
#include <entt/fwd.hpp>

namespace edyn {

using entity_set = std::unordered_set<entt::entity>;

}

#endif // EDYN_UTIL_ENTITY_COLLECTION_HPP