#ifndef EDYN_COLLISION_QUERY_AABB_HPP
#define EDYN_COLLISION_QUERY_AABB_HPP

#include "edyn/collision/broadphase.hpp"
#include "edyn/comp/aabb.hpp"
#include <entt/entity/registry.hpp>

namespace edyn {

template<typename Func>
void query_island_aabb(entt::registry &registry, const AABB &aabb, Func func) {
    auto &bphase = registry.ctx().at<broadphase>();
    bphase.query_islands(aabb, func);
}

using query_aabb_id_type = unsigned;
using query_aabb_delegate_type = entt::delegate<void(query_aabb_id_type, const std::vector<entt::entity> &)>;

query_aabb_id_type query_island_aabb_async(entt::registry &registry, const AABB &aabb,
                                           const query_aabb_delegate_type &delegate);

}

#endif // EDYN_COLLISION_QUERY_AABB_HPP
