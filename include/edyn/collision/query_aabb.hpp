#ifndef EDYN_COLLISION_QUERY_AABB_HPP
#define EDYN_COLLISION_QUERY_AABB_HPP

#include "edyn/collision/broadphase.hpp"
#include "edyn/comp/aabb.hpp"
#include <entt/entity/registry.hpp>

namespace edyn {

template<typename Func>
void query_procedural_aabb(entt::registry &registry, const AABB &aabb, Func func) {
    auto &bphase = registry.ctx().get<broadphase>();
    bphase.query_procedural(aabb, func);
}

template<typename Func>
void query_non_procedural_aabb(entt::registry &registry, const AABB &aabb, Func func) {
    auto &bphase = registry.ctx().get<broadphase>();
    bphase.query_non_procedural(aabb, func);
}

template<typename Func>
void query_island_aabb(entt::registry &registry, const AABB &aabb, Func func) {
    auto &bphase = registry.ctx().get<broadphase>();
    bphase.query_islands(aabb, func);
}

struct query_aabb_result {
    std::vector<entt::entity> procedural_entities;
    std::vector<entt::entity> non_procedural_entities;
    std::vector<entt::entity> island_entities;
};

using query_aabb_id_type = unsigned;
using query_aabb_delegate_type = entt::delegate<void(query_aabb_id_type, const query_aabb_result &)>;

query_aabb_id_type query_aabb_async(entt::registry &registry, const AABB &aabb,
                                    const query_aabb_delegate_type &delegate,
                                    bool query_procedural,
                                    bool query_non_procedural,
                                    bool query_islands);

query_aabb_id_type query_aabb_of_interest_async(entt::registry &registry, const AABB &aabb,
                                                const query_aabb_delegate_type &delegate);

}

#endif // EDYN_COLLISION_QUERY_AABB_HPP
