#ifndef EDYN_UTIL_RAGDOLL_HPP
#define EDYN_UTIL_RAGDOLL_HPP

#include "edyn/math/vector3.hpp"
#include "edyn/math/quaternion.hpp"
#include <entt/entity/fwd.hpp>

namespace edyn {

struct ragdoll_simple_def {
    vector3 position;
    quaternion orientation;

    scalar height;
    scalar weight;
};

struct ragdoll_def {
    vector3 position;
    quaternion orientation;

    scalar head_mass;
    scalar neck_mass;
    scalar torso_upper_mass;
    scalar torso_middle_mass;
    scalar torso_lower_mass;
    scalar hip_mass;
    scalar leg_upper_mass;
    scalar leg_lower_mass;
    scalar foot_mass;
    scalar shoulder_mass;
    scalar arm_upper_mass;
    scalar arm_lower_mass;
    scalar hand_mass;

    vector3 head_size;
    vector3 neck_size;
    vector3 torso_upper_size;
    vector3 torso_middle_size;
    vector3 torso_lower_size;
    vector3 hip_size;
    vector3 leg_upper_size;
    vector3 leg_lower_size;
    vector3 foot_size;
    vector3 shoulder_size;
    vector3 arm_upper_size;
    vector3 arm_lower_size;
    vector3 hand_size;

    scalar restitution{0};
    scalar friction{0.5};
};

struct ragdoll_entities {
    entt::entity head;
    entt::entity neck;
    entt::entity torso_upper;
    entt::entity torso_middle;
    entt::entity torso_lower;
    entt::entity hip;
    entt::entity leg_upper_left;
    entt::entity leg_upper_right;
    entt::entity leg_lower_left;
    entt::entity leg_lower_right;
    entt::entity foot_left;
    entt::entity foot_right;
    entt::entity shoulder_left;
    entt::entity shoulder_right;
    entt::entity arm_upper_left;
    entt::entity arm_upper_right;
    entt::entity arm_lower_left;
    entt::entity arm_lower_right;
    entt::entity arm_twist_left;
    entt::entity arm_twist_right;
    entt::entity hand_left;
    entt::entity hand_right;

    entt::entity hip_torso_lower_cone;
    entt::entity hip_torso_lower_cvjoint;

    entt::entity torso_lower_torso_middle_cone;
    entt::entity torso_lower_torso_middle_cvjoint;

    entt::entity torso_middle_torso_upper_cone;
    entt::entity torso_middle_torso_upper_cvjoint;

    entt::entity torso_upper_neck_cone;
    entt::entity torso_upper_neck_cvjoint;

    entt::entity neck_head_cone;
    entt::entity neck_head_cvjoint;

    entt::entity hip_upper_leg_left_cone;
    entt::entity hip_upper_leg_left_cvjoint;
    entt::entity hip_upper_leg_right_cone;
    entt::entity hip_upper_leg_right_cvjoint;

    entt::entity knee_left;
    entt::entity knee_right;

    entt::entity ankle_left_cone;
    entt::entity ankle_left_cvjoint;
    entt::entity ankle_right_cone;
    entt::entity ankle_right_cvjoint;
};

ragdoll_def make_ragdoll_def_from_simple(const ragdoll_simple_def &def);

ragdoll_entities make_ragdoll(entt::registry &registry, const ragdoll_simple_def &def);
ragdoll_entities make_ragdoll(entt::registry &registry, const ragdoll_def &def);

}

#endif // EDYN_UTIL_RAGDOLL_HPP
