#include "edyn/collision/query_aabb.hpp"
#include "edyn/simulation/stepper_async.hpp"

namespace edyn {

query_aabb_id_type query_island_aabb_async(entt::registry &registry, const AABB &aabb,
                                           const query_aabb_delegate_type &delegate,
                                           bool query_procedural,
                                           bool query_non_procedural,
                                           bool query_islands) {
    auto &stepper = registry.ctx().get<stepper_async>();
    return stepper.query_aabb(aabb, delegate, query_procedural, query_non_procedural, query_islands);
}


query_aabb_id_type query_aabb_of_interest_async(entt::registry &registry, const AABB &aabb,
                                                const query_aabb_delegate_type &delegate) {
    auto &stepper = registry.ctx().get<stepper_async>();
    return stepper.query_aabb_of_interest(aabb, delegate);
}

}
