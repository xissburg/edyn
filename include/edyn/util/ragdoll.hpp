#ifndef EDYN_UTIL_RAGDOLL_HPP
#define EDYN_UTIL_RAGDOLL_HPP

#include "edyn/math/vector3.hpp"
#include "edyn/math/quaternion.hpp"
#include <entt/entity/fwd.hpp>

namespace edyn {

struct ragdoll_simple_def {
    vector3 position{vector3_zero};
    quaternion orientation{quaternion_identity};

    scalar height{scalar(1.7)};
    scalar weight{scalar(72)};

    scalar restitution{0};
    scalar friction{0.5};
};

struct ragdoll_def {
    vector3 position{vector3_zero};
    quaternion orientation{quaternion_identity};

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

    entt::entity hip_torso_lower_constraint;
    entt::entity torso_lower_torso_middle_constraint;
    entt::entity torso_middle_torso_upper_constraint;
    entt::entity torso_upper_neck_constraint;
    entt::entity neck_head_constraint;

    entt::entity hip_upper_leg_left_constraint;
    entt::entity hip_upper_leg_right_constraint;

    entt::entity knee_left_hinge;
    entt::entity knee_right_hinge;

    entt::entity ankle_left_constraint;
    entt::entity ankle_right_constraint;

    entt::entity torso_upper_shoulder_left_constraint;
    entt::entity torso_upper_shoulder_right_constraint;

    entt::entity shoulder_arm_upper_left_constraint;
    entt::entity shoulder_arm_upper_right_constraint;

    entt::entity elbow_left_hinge;
    entt::entity elbow_right_hinge;

    entt::entity arm_twist_left_hinge;
    entt::entity arm_twist_right_hinge;

    entt::entity wrist_left_constraint;
    entt::entity wrist_right_constraint;
};

/**
 * @brief Constructs a full rag doll definition from a simple definition.
 * @param def Simple rag doll definition.
 * @return Full rag doll definition.
 */
ragdoll_def make_ragdoll_def_from_simple(const ragdoll_simple_def &def);

/**
 * @brief Creates rigid bodies and constraints which form a rag doll.
 * @param registry Data source.
 * @param def Simple rag doll definition.
 * @return All entities created.
 */
ragdoll_entities make_ragdoll(entt::registry &registry, const ragdoll_simple_def &def);

/**
 * @brief Creates rigid bodies and constraints which form a rag doll.
 * @param registry Data source.
 * @param def A full rag doll definition.
 * @return All entities created.
 */
ragdoll_entities make_ragdoll(entt::registry &registry, const ragdoll_def &def);

}

#endif // EDYN_UTIL_RAGDOLL_HPP
