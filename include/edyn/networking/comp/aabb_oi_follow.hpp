#ifndef EDYN_NETWORKING_COMP_AABB_OI_FOLLOW_HPP
#define EDYN_NETWORKING_COMP_AABB_OI_FOLLOW_HPP

namespace edyn {

/**
 * @brief The center of the AABB of interest of the remote client which has
 * this component will be set to match the position of the referred entity.
 */
struct aabb_oi_follow {
    entt::entity entity;
};

}

#endif // EDYN_NETWORKING_COMP_AABB_OI_FOLLOW_HPP
