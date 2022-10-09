#ifndef EDYN_CORE_ENTITY_PAIR
#define EDYN_CORE_ENTITY_PAIR

#include <vector>
#include <utility>
#include <entt/entity/fwd.hpp>

namespace edyn {

using entity_pair = std::pair<entt::entity, entt::entity>;
using entity_pair_vector = std::vector<entity_pair>;

}

#endif // EDYN_CORE_ENTITY_PAIR
