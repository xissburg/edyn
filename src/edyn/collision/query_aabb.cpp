#include "edyn/collision/query_aabb.hpp"
#include "edyn/simulation/stepper_async.hpp"

namespace edyn {

query_aabb_id_type query_island_aabb_async(entt::registry &registry, const AABB &aabb,
                                           const query_aabb_delegate_type &delegate) {
    auto &stepper = registry.ctx().at<stepper_async>();
    return stepper.query_island_aabb(aabb, delegate);
}

}
