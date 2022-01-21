#include "edyn/util/ragdoll.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/constraints/cone_constraint.hpp"
#include "edyn/constraints/cvjoint_constraint.hpp"
#include "edyn/constraints/hinge_constraint.hpp"
#include "edyn/math/transform.hpp"
#include "edyn/util/constraint_util.hpp"
#include "edyn/util/moment_of_inertia.hpp"
#include "edyn/util/rigidbody.hpp"
#include "edyn/util/exclude_collision.hpp"

namespace edyn {

ragdoll_def make_ragdoll_def_from_simple(const ragdoll_simple_def &simple_def) {
    auto rag_def = ragdoll_def{};

    rag_def.position = simple_def.position;
    rag_def.orientation = simple_def.orientation;
    rag_def.friction = simple_def.friction;
    rag_def.restitution = simple_def.restitution;

    rag_def.head_mass = simple_def.weight * 4 / 72;
    rag_def.neck_mass = simple_def.weight * 2 / 72;
    rag_def.torso_upper_mass = simple_def.weight * 7 / 72;
    rag_def.torso_middle_mass = simple_def.weight * 6 / 72;
    rag_def.torso_lower_mass = simple_def.weight * 5 / 72;
    rag_def.hip_mass = simple_def.weight * 3 / 72;
    rag_def.leg_upper_mass = simple_def.weight * 8 / 72;
    rag_def.leg_lower_mass = simple_def.weight * 7 / 72;
    rag_def.foot_mass = simple_def.weight * 1 / 72;
    rag_def.shoulder_mass = simple_def.weight * 1.5 / 72;
    rag_def.arm_upper_mass = simple_def.weight * 2.5 / 72;
    rag_def.arm_lower_mass = simple_def.weight * 2 / 72;
    rag_def.hand_mass = simple_def.weight * 0.5 / 72;

    scalar vertical_scale = simple_def.height / 1.7;
    // Scale horizontally at a lower rate.
    scalar horizontal_scale = 0.2 + vertical_scale * 0.8;
    auto scale = vector3{horizontal_scale, vertical_scale, horizontal_scale};

    rag_def.head_size         = scale * 2 * vector3{0.075, 0.09, 0.105};
    rag_def.neck_size         = scale * 2 * vector3{0.06, 0.065, 0.06};
    rag_def.torso_upper_size  = scale * 2 * vector3{0.17, 0.108, 0.095};
    rag_def.torso_middle_size = scale * 2 * vector3{0.151, 0.084, 0.07};
    rag_def.torso_lower_size  = scale * 2 * vector3{0.155, 0.065, 0.086};
    rag_def.hip_size          = scale * 2 * vector3{0.17, 0.07, 0.1};
    rag_def.leg_upper_size    = scale * 2 * vector3{0.075, 0.205, 0.075};
    rag_def.leg_lower_size    = scale * 2 * vector3{0.06, 0.205, 0.06};
    rag_def.foot_size         = scale * 2 * vector3{0.05, 0.04, 0.13};

    // Arms are initially oriented horizontally, thus flip the scale.
    scale = vector3{vertical_scale, horizontal_scale, horizontal_scale};
    rag_def.arm_upper_size    = scale * 2 * vector3{0.135, 0.05, 0.05};
    rag_def.arm_lower_size    = scale * 2 * vector3{0.135, 0.04, 0.04};
    rag_def.hand_size         = scale * 2 * vector3{0.065, 0.045, 0.045};

    return rag_def;
}

ragdoll_entities make_ragdoll(entt::registry &registry, const ragdoll_simple_def &def) {
    return make_ragdoll(registry, make_ragdoll_def_from_simple(def));
}

ragdoll_entities make_ragdoll(entt::registry &registry, const ragdoll_def &rag_def) {
    auto entities = ragdoll_entities{};

    /* Head */ {
        auto def = rigidbody_def();
        def.material->restitution = rag_def.restitution;
        def.material->friction = rag_def.friction;
        def.mass = rag_def.head_mass;
        def.shape = box_shape{rag_def.head_size / 2};
        def.update_inertia();
        auto pos_y =
            rag_def.head_size.y / 2 +
            rag_def.neck_size.y * scalar(0.627) +
            rag_def.torso_upper_size.y +
            rag_def.torso_middle_size.y +
            rag_def.torso_lower_size.y +
            rag_def.hip_size.y / 2;
        def.position = to_world_space({0, pos_y, -0.025}, rag_def.position, rag_def.orientation);
        def.orientation = rag_def.orientation;
        entities.head = make_rigidbody(registry, def);
    }

    /* Neck */ {
        auto def = rigidbody_def();
        def.material->restitution = rag_def.restitution;
        def.material->friction = rag_def.friction;
        def.mass = rag_def.neck_mass;
        def.shape = box_shape{rag_def.neck_size / 2};
        def.update_inertia();
        auto pos_y =
            rag_def.neck_size.y * scalar(0.627) / 2 +
            rag_def.torso_upper_size.y +
            rag_def.torso_middle_size.y +
            rag_def.torso_lower_size.y +
            rag_def.hip_size.y / 2;
        def.position = to_world_space({0, pos_y, 0}, rag_def.position, rag_def.orientation);
        def.orientation = rag_def.orientation;
        entities.neck = make_rigidbody(registry, def);
    }

    /* Upper torso */ {
        auto def = rigidbody_def();
        def.material->restitution = rag_def.restitution;
        def.material->friction = rag_def.friction;
        def.mass = rag_def.torso_upper_mass;
        def.shape = box_shape{rag_def.torso_upper_size / 2};
        def.update_inertia();
        auto pos_y =
            rag_def.torso_upper_size.y / 2 +
            rag_def.torso_middle_size.y +
            rag_def.torso_lower_size.y +
            rag_def.hip_size.y / 2;
        def.position = to_world_space({0, pos_y, 0}, rag_def.position, rag_def.orientation);
        def.orientation = rag_def.orientation;
        entities.torso_upper = make_rigidbody(registry, def);
    }

    /* Mid torso */ {
        auto def = rigidbody_def();
        def.material->restitution = rag_def.restitution;
        def.material->friction = rag_def.friction;
        def.mass = rag_def.torso_middle_mass;
        def.shape = box_shape{rag_def.torso_middle_size / 2};
        def.update_inertia();
        auto pos_y =
            rag_def.torso_middle_size.y / 2 +
            rag_def.torso_lower_size.y +
            rag_def.hip_size.y / 2;
        def.position = to_world_space({0, pos_y, 0}, rag_def.position, rag_def.orientation);
        def.orientation = rag_def.orientation;
        entities.torso_middle = make_rigidbody(registry, def);
    }

    /* Lower torso */ {
        auto def = rigidbody_def();
        def.material->restitution = rag_def.restitution;
        def.material->friction = rag_def.friction;
        def.mass = rag_def.torso_lower_mass;
        def.shape = box_shape{rag_def.torso_lower_size / 2};
        def.update_inertia();
        auto pos_y =
            rag_def.torso_lower_size.y / 2 +
            rag_def.hip_size.y / 2;
        def.position = to_world_space({0, pos_y, 0}, rag_def.position, rag_def.orientation);
        def.orientation = rag_def.orientation;
        entities.torso_lower = make_rigidbody(registry, def);
    }

    /* Hip */ {
        auto def = rigidbody_def();
        def.material->restitution = rag_def.restitution;
        def.material->friction = rag_def.friction;
        def.mass = rag_def.hip_mass;
        def.shape = box_shape{rag_def.hip_size / 2};
        def.update_inertia();
        def.position = rag_def.position;
        def.orientation = rag_def.orientation;
        entities.hip = make_rigidbody(registry, def);
    }

    auto leg_pos_x = rag_def.hip_size.x / 2 - (rag_def.leg_upper_size.x - scalar(0.0072)) / 2;

    /* Upper legs */
    for (auto i = 0; i < 2; ++i) {
        auto def = rigidbody_def();
        def.material->restitution = rag_def.restitution;
        def.material->friction = rag_def.friction;
        def.mass = rag_def.leg_upper_mass;
        def.shape = box_shape{rag_def.leg_upper_size / 2};
        def.update_inertia();

        auto pos_x = leg_pos_x * to_sign(i == 0);
        auto pos_y = -rag_def.leg_upper_size.y / 2;

        def.position = to_world_space({pos_x, pos_y, 0}, rag_def.position, rag_def.orientation);
        def.orientation = rag_def.orientation;

        *std::array{&entities.leg_upper_left, &entities.leg_upper_right}[i] = make_rigidbody(registry, def);
    }

    /* Lower legs */
    for (auto i = 0; i < 2; ++i) {
        auto def = rigidbody_def();
        def.material->restitution = rag_def.restitution;
        def.material->friction = rag_def.friction;
        def.mass = rag_def.leg_lower_mass;
        def.shape = box_shape{rag_def.leg_lower_size / 2};
        def.update_inertia();

        auto pos_x = leg_pos_x * to_sign(i == 0);
        auto pos_y = -(rag_def.leg_upper_size.y + rag_def.leg_lower_size.y / 2);

        def.position = to_world_space({pos_x, pos_y, 0}, rag_def.position, rag_def.orientation);
        def.orientation = rag_def.orientation;

        *std::array{&entities.leg_lower_left, &entities.leg_lower_right}[i] = make_rigidbody(registry, def);
    }

    /* Feet */
    for (auto i = 0; i < 2; ++i) {
        auto def = rigidbody_def();
        def.material->restitution = rag_def.restitution;
        def.material->friction = rag_def.friction;
        def.mass = rag_def.foot_mass;
        def.shape = box_shape{rag_def.foot_size / 2};
        def.update_inertia();

        auto pos_x = leg_pos_x * to_sign(i == 0);
        auto pos_y = -(rag_def.leg_upper_size.y + rag_def.leg_lower_size.y + rag_def.foot_size.y / 2);
        auto pos_z = -rag_def.leg_lower_size.z / 2;

        def.position = to_world_space({pos_x, pos_y, pos_z}, rag_def.position, rag_def.orientation);
        def.orientation = rag_def.orientation;

        *std::array{&entities.foot_left, &entities.foot_right}[i] = make_rigidbody(registry, def);
    }

    auto torso_upper_top =
        rag_def.torso_upper_size.y  +
        rag_def.torso_middle_size.y +
        rag_def.torso_lower_size.y +
        rag_def.hip_size.y / 2;

    auto shoulder_size = vector3{
        rag_def.torso_upper_size.x * scalar(0.352),
        rag_def.arm_upper_size.y, rag_def.arm_upper_size.z};

    auto rot_z_pi = quaternion_axis_angle({0, 0, 1}, pi);

    /* Shoulders */
    for (auto i = 0; i < 2; ++i) {
        auto def = rigidbody_def();
        def.material->restitution = rag_def.restitution;
        def.material->friction = rag_def.friction;
        def.mass = rag_def.shoulder_mass;
        def.inertia = diagonal_matrix(moment_of_inertia_solid_box(def.mass, shoulder_size));
        auto pos_x = rag_def.torso_upper_size.x / 2 * scalar(0.65) * to_sign(i == 0);
        auto pos_y = torso_upper_top - rag_def.arm_upper_size.y / 2;

        def.position = to_world_space({pos_x, pos_y, 0}, rag_def.position, rag_def.orientation);
        def.orientation = i == 0 ? rag_def.orientation : rag_def.orientation * rot_z_pi;

        *std::array{&entities.shoulder_left, &entities.shoulder_right}[i] = make_rigidbody(registry, def);
    }

    /* Upper arms */
    for (auto i = 0; i < 2; ++i) {
        auto def = rigidbody_def();
        def.material->restitution = rag_def.restitution;
        def.material->friction = rag_def.friction;
        def.mass = rag_def.arm_upper_mass;
        def.shape = box_shape{rag_def.arm_upper_size / 2};
        def.update_inertia();

        auto pos_x = (rag_def.torso_upper_size.x / 2 +
                      rag_def.arm_upper_size.x / 2) * to_sign(i == 0);
        auto pos_y = torso_upper_top - rag_def.arm_upper_size.y / 2;

        def.position = to_world_space({pos_x, pos_y, 0}, rag_def.position, rag_def.orientation);
        def.orientation = i == 0 ? rag_def.orientation : rag_def.orientation * rot_z_pi;

        *std::array{&entities.arm_upper_left, &entities.arm_upper_right}[i] = make_rigidbody(registry, def);
    }

    /* Lower arms */
    for (auto i = 0; i < 2; ++i) {
        auto def = rigidbody_def();
        def.material->restitution = rag_def.restitution;
        def.material->friction = rag_def.friction;
        def.mass = rag_def.arm_lower_mass / 2; // Split mass with forearm twist body.
        def.shape = box_shape{rag_def.arm_lower_size / 2};
        def.update_inertia();

        auto pos_x = (rag_def.torso_upper_size.x / 2 +
                      rag_def.arm_upper_size.x +
                      rag_def.arm_lower_size.x / 2) * to_sign(i == 0);
        auto pos_y = torso_upper_top - rag_def.arm_upper_size.y / 2;

        def.position = to_world_space({pos_x, pos_y, 0}, rag_def.position, rag_def.orientation);
        def.orientation = i == 0 ? rag_def.orientation : rag_def.orientation * rot_z_pi;

        *std::array{&entities.arm_lower_left, &entities.arm_lower_right}[i] = make_rigidbody(registry, def);

        def.shape = {};
        *std::array{&entities.arm_twist_left, &entities.arm_twist_right}[i] = make_rigidbody(registry, def);
    }

    /* Hands */
    for (auto i = 0; i < 2; ++i) {
        auto def = rigidbody_def();
        def.material->restitution = rag_def.restitution;
        def.material->friction = rag_def.friction;
        def.mass = rag_def.hand_mass;
        def.shape = box_shape{rag_def.hand_size / 2};
        def.update_inertia();

        auto pos_x = (rag_def.torso_upper_size.x / 2 +
                      rag_def.arm_upper_size.x +
                      rag_def.arm_lower_size.x +
                      rag_def.hand_size.x / 2) * to_sign(i == 0);
        auto pos_y = torso_upper_top - rag_def.arm_upper_size.y / 2 -
                     (rag_def.hand_size.y - rag_def.arm_lower_size.y) / 2;
        auto pos_z = -(rag_def.hand_size.z - rag_def.arm_lower_size.z) / 2;

        def.position = to_world_space({pos_x, pos_y, pos_z}, rag_def.position, rag_def.orientation);
        def.orientation = i == 0 ? rag_def.orientation : rag_def.orientation * rot_z_pi;

        *std::array{&entities.hand_left, &entities.hand_right}[i] = make_rigidbody(registry, def);
    }

    // Configure collision exclusions.
    exclude_collision(registry, entities.hip, entities.torso_lower);
    exclude_collision(registry, entities.torso_middle, entities.torso_lower);
    exclude_collision(registry, entities.torso_middle, entities.torso_upper);
    exclude_collision(registry, entities.neck, entities.torso_upper);
    exclude_collision(registry, entities.neck, entities.head);
    exclude_collision(registry, entities.hip, entities.leg_upper_left);
    exclude_collision(registry, entities.hip, entities.leg_upper_right);
    exclude_collision(registry, entities.torso_lower, entities.leg_upper_left);
    exclude_collision(registry, entities.torso_lower, entities.leg_upper_right);
    exclude_collision(registry, entities.leg_lower_left, entities.leg_upper_left);
    exclude_collision(registry, entities.leg_lower_right, entities.leg_upper_right);
    exclude_collision(registry, entities.leg_lower_left, entities.foot_left);
    exclude_collision(registry, entities.leg_lower_right, entities.foot_right);
    exclude_collision(registry, entities.torso_upper, entities.shoulder_left);
    exclude_collision(registry, entities.torso_upper, entities.shoulder_right);
    exclude_collision(registry, entities.torso_upper, entities.arm_upper_left);
    exclude_collision(registry, entities.torso_upper, entities.arm_upper_right);
    exclude_collision(registry, entities.arm_lower_left, entities.arm_upper_left);
    exclude_collision(registry, entities.arm_lower_right, entities.arm_upper_right);
    exclude_collision(registry, entities.arm_lower_left, entities.hand_left);
    exclude_collision(registry, entities.arm_lower_right, entities.hand_right);

    /* Hip-lower torso */ {
        entities.hip_torso_lower_constraint = registry.create();

        auto &cone_con = make_constraint<cone_constraint>(
            entities.hip_torso_lower_constraint, registry, entities.hip, entities.torso_lower);
        cone_con.pivot[0] = {0, rag_def.hip_size.y / 2, 0};
        cone_con.pivot[1] = {0, rag_def.torso_lower_size.y, 0};
        cone_con.frame = matrix3x3_columns(vector3_y, -vector3_x, vector3_z);
        cone_con.span_tan[0] = std::tan(to_radians(10));
        cone_con.span_tan[1] = std::tan(to_radians(20));
        cone_con.bump_stop_stiffness = 5000;
        cone_con.bump_stop_length = 0.05;

        auto &cvjoint = make_constraint<cvjoint_constraint>(
            entities.hip_torso_lower_constraint, registry, entities.hip, entities.torso_lower);
        cvjoint.pivot[0] = cone_con.pivot[0];
        cvjoint.pivot[1] = {0, -rag_def.torso_lower_size.y / 2, 0};
        cvjoint.frame[0] = matrix3x3_columns(vector3_y, -vector3_x, vector3_z);
        cvjoint.frame[1] = matrix3x3_columns(vector3_y, -vector3_x, vector3_z);
        cvjoint.twist_min = to_radians(-12);
        cvjoint.twist_max = -cvjoint.twist_min;
        cvjoint.reset_angle(
            registry.get<orientation>(entities.hip),
            registry.get<orientation>(entities.torso_lower));
        cvjoint.twist_friction_torque = cvjoint.bend_friction_torque = to_Nm_per_radian(0.02);
        cvjoint.twist_damping = cvjoint.bend_damping = to_Nm_per_radian(0.2);
        cvjoint.twist_bump_stop_angle = to_radians(4);
        cvjoint.twist_bump_stop_stiffness = to_Nm_per_radian(5);
    }

    /* Lower torso-Mid torso */ {
        entities.torso_lower_torso_middle_constraint = registry.create();

        auto &cone_con = make_constraint<cone_constraint>(
            entities.torso_lower_torso_middle_constraint, registry, entities.torso_lower, entities.torso_middle);
        cone_con.pivot[0] = {0, rag_def.torso_lower_size.y / 2, 0};
        cone_con.pivot[1] = {0, rag_def.torso_middle_size.y, 0};
        cone_con.frame = matrix3x3_columns(vector3_y, -vector3_x, vector3_z);
        cone_con.span_tan[0] = std::tan(to_radians(16));
        cone_con.span_tan[1] = std::tan(to_radians(30));
        cone_con.bump_stop_stiffness = 5000;
        cone_con.bump_stop_length = 0.05;

        auto &cvjoint = make_constraint<cvjoint_constraint>(
            entities.torso_lower_torso_middle_constraint, registry, entities.torso_lower, entities.torso_middle);
        cvjoint.pivot[0] = cone_con.pivot[0];
        cvjoint.pivot[1] = {0, -rag_def.torso_middle_size.y / 2, 0};
        cvjoint.frame[0] = matrix3x3_columns(vector3_y, -vector3_x, vector3_z);
        cvjoint.frame[1] = matrix3x3_columns(vector3_y, -vector3_x, vector3_z);
        cvjoint.twist_min = to_radians(-18);
        cvjoint.twist_max = -cvjoint.twist_min;
        cvjoint.reset_angle(
            registry.get<orientation>(entities.torso_lower),
            registry.get<orientation>(entities.torso_middle));
        cvjoint.twist_friction_torque = cvjoint.bend_friction_torque = to_Nm_per_radian(0.02);
        cvjoint.twist_damping = cvjoint.bend_damping = to_Nm_per_radian(0.2);
        cvjoint.twist_bump_stop_angle = to_radians(4);
        cvjoint.twist_bump_stop_stiffness = to_Nm_per_radian(5);
    }

    /* Mid torso-upper torso */ {
        entities.torso_middle_torso_upper_constraint = registry.create();

        auto &cone_con = make_constraint<cone_constraint>(
            entities.torso_middle_torso_upper_constraint, registry, entities.torso_middle, entities.torso_upper);
        cone_con.pivot[0] = {0, rag_def.torso_middle_size.y / 2, 0};
        cone_con.pivot[1] = {0, rag_def.torso_upper_size.y, 0};
        cone_con.frame = matrix3x3_columns(vector3_y, -vector3_x, vector3_z);
        cone_con.span_tan[0] = std::tan(to_radians(18));
        cone_con.span_tan[1] = std::tan(to_radians(32));
        cone_con.bump_stop_stiffness = 5000;
        cone_con.bump_stop_length = 0.05;

        auto &cvjoint = make_constraint<cvjoint_constraint>(
            entities.torso_middle_torso_upper_constraint, registry, entities.torso_middle, entities.torso_upper);
        cvjoint.pivot[0] = cone_con.pivot[0];
        cvjoint.pivot[1] = {0, -rag_def.torso_upper_size.y / 2, 0};
        cvjoint.frame[0] = matrix3x3_columns(vector3_y, -vector3_x, vector3_z);
        cvjoint.frame[1] = matrix3x3_columns(vector3_y, -vector3_x, vector3_z);
        cvjoint.twist_min = to_radians(-10);
        cvjoint.twist_max = -cvjoint.twist_min;
        cvjoint.reset_angle(
            registry.get<orientation>(entities.torso_middle),
            registry.get<orientation>(entities.torso_upper));
        cvjoint.twist_friction_torque = cvjoint.bend_friction_torque = to_Nm_per_radian(0.02);
        cvjoint.twist_damping = cvjoint.bend_damping = to_Nm_per_radian(0.2);
        cvjoint.twist_bump_stop_angle = to_radians(4);
        cvjoint.twist_bump_stop_stiffness = to_Nm_per_radian(5);
    }

    /* Upper torso-neck */ {
        entities.torso_upper_neck_constraint = registry.create();

        auto &cone_con = make_constraint<cone_constraint>(
            entities.torso_upper_neck_constraint, registry, entities.torso_upper, entities.neck);
        cone_con.pivot[0] = {0, rag_def.torso_upper_size.y / 2, 0};
        cone_con.pivot[1] = {0, rag_def.neck_size.y, 0};
        cone_con.frame = matrix3x3_columns(vector3_y, -vector3_x, vector3_z);
        cone_con.span_tan[0] = std::tan(to_radians(16));
        cone_con.span_tan[1] = std::tan(to_radians(32));
        cone_con.bump_stop_stiffness = 3000;
        cone_con.bump_stop_length = 0.05;

        auto &cvjoint = make_constraint<cvjoint_constraint>(
            entities.torso_upper_neck_constraint, registry, entities.torso_upper, entities.neck);
        cvjoint.pivot[0] = cone_con.pivot[0];
        cvjoint.pivot[1] = {0, -rag_def.neck_size.y * scalar(0.33), 0};
        cvjoint.frame[0] = matrix3x3_columns(vector3_y, -vector3_x, vector3_z);
        cvjoint.frame[1] = matrix3x3_columns(vector3_y, -vector3_x, vector3_z);
        cvjoint.twist_min = to_radians(-30);
        cvjoint.twist_max = -cvjoint.twist_min;
        cvjoint.reset_angle(
            registry.get<orientation>(entities.torso_upper),
            registry.get<orientation>(entities.neck));
        cvjoint.twist_friction_torque = cvjoint.bend_friction_torque = to_Nm_per_radian(0.02);
        cvjoint.twist_damping = cvjoint.bend_damping = to_Nm_per_radian(0.2);
        cvjoint.twist_bump_stop_angle = to_radians(4);
        cvjoint.twist_bump_stop_stiffness = to_Nm_per_radian(5);
    }

    /* Neck-head */ {
        entities.neck_head_constraint = registry.create();

        auto &cone_con = make_constraint<cone_constraint>(
            entities.neck_head_constraint, registry, entities.neck, entities.head);
        cone_con.pivot[0] = {0, rag_def.neck_size.y / 2, 0};
        cone_con.pivot[1] = {0, rag_def.head_size.y, 0.025};
        cone_con.frame = matrix3x3_columns(vector3_y, -vector3_x, vector3_z);
        cone_con.span_tan[0] = std::tan(to_radians(16));
        cone_con.span_tan[1] = std::tan(to_radians(32));
        cone_con.bump_stop_stiffness = 5000;
        cone_con.bump_stop_length = 0.05;

        auto &cvjoint = make_constraint<cvjoint_constraint>(
            entities.neck_head_constraint, registry, entities.neck, entities.head);
        cvjoint.pivot[0] = cone_con.pivot[0];
        cvjoint.pivot[1] = {0, -(rag_def.head_size.y / 2 - rag_def.neck_size.y * scalar(0.2)), 0.025};
        cvjoint.frame[0] = matrix3x3_columns(vector3_y, -vector3_x, vector3_z);
        cvjoint.frame[1] = matrix3x3_columns(vector3_y, -vector3_x, vector3_z);
        cvjoint.twist_min = to_radians(-30);
        cvjoint.twist_max = -cvjoint.twist_min;
        cvjoint.reset_angle(
            registry.get<orientation>(entities.neck),
            registry.get<orientation>(entities.head));
        cvjoint.twist_friction_torque = cvjoint.bend_friction_torque = to_Nm_per_radian(0.02);
        cvjoint.twist_damping = cvjoint.bend_damping = to_Nm_per_radian(0.2);
        cvjoint.twist_bump_stop_angle = to_radians(4);
        cvjoint.twist_bump_stop_stiffness = to_Nm_per_radian(5);
    }

    /* Hip-upper legs */
    for (auto i = 0; i < 2; ++i) {
        auto leg = std::array{entities.leg_upper_left, entities.leg_upper_right}[i];
        scalar side = to_sign(i == 0);
        auto cone_rot =
            quaternion_axis_angle({1, 0, 0}, to_radians(50)) *
            quaternion_axis_angle({0, 0, 1}, to_radians(10 * side));
        auto con_entity = registry.create();

        auto &cone_con = make_constraint<cone_constraint>(con_entity, registry, entities.hip, leg);
        cone_con.pivot[0] = {side * (rag_def.hip_size.x / 2 - (rag_def.leg_upper_size.x - scalar(0.0072)) / 2), 0, 0};
        cone_con.pivot[1] = {0, -rag_def.leg_upper_size.y, 0};
        cone_con.frame = matrix3x3_columns(
            rotate(cone_rot, -vector3_y),
            rotate(cone_rot, vector3_x),
            rotate(cone_rot, -vector3_z));
        cone_con.span_tan[0] = std::tan(to_radians(45));
        cone_con.span_tan[1] = std::tan(to_radians(70));
        cone_con.bump_stop_stiffness = 5000;
        cone_con.bump_stop_length = 0.05;

        auto &cvjoint = make_constraint<cvjoint_constraint>(con_entity, registry, entities.hip, leg);
        cvjoint.pivot[0] = cone_con.pivot[0];
        cvjoint.pivot[1] = {0, rag_def.leg_upper_size.y / 2, 0};
        cvjoint.frame[0] = matrix3x3_columns(vector3_y, vector3_x, -vector3_z);
        cvjoint.frame[1] = matrix3x3_columns(vector3_y, vector3_x, -vector3_z);
        cvjoint.twist_min = to_radians(leg == entities.leg_upper_left ? -80 : -15);
        cvjoint.twist_max = to_radians(leg == entities.leg_upper_left ? 15 : 80);

        cvjoint.reset_angle(
            registry.get<orientation>(entities.hip),
            registry.get<orientation>(leg));
        cvjoint.twist_friction_torque = cvjoint.bend_friction_torque = to_Nm_per_radian(0.02);
        cvjoint.twist_damping = cvjoint.bend_damping = to_Nm_per_radian(0.2);
        cvjoint.twist_bump_stop_angle = to_radians(4);
        cvjoint.twist_bump_stop_stiffness = to_Nm_per_radian(5);

        *std::array{
            &entities.hip_upper_leg_left_constraint,
            &entities.hip_upper_leg_right_constraint
        }[i] = con_entity;
    }

    /* Knees */
    for (auto i = 0; i < 2; ++i) {
        const auto legs = std::array{
            std::make_pair(entities.leg_upper_left, entities.leg_lower_left),
            std::make_pair(entities.leg_upper_right, entities.leg_lower_right)
        }[i];

        auto [hinge_ent, hinge] = make_constraint<hinge_constraint>(registry, legs.first, legs.second);
        hinge.pivot[0] = {0, -rag_def.leg_upper_size.y / 2, 0};
        hinge.pivot[1] = {0, rag_def.leg_lower_size.y / 2, 0};
        hinge.set_axes({1, 0, 0}, {1, 0, 0});
        hinge.angle_min = to_radians(-140);
        hinge.angle_max = 0;
        hinge.damping = 2;
        hinge.friction_torque = 1;
        hinge.bump_stop_angle = to_radians(10);
        hinge.bump_stop_stiffness = 30;
        hinge.reset_angle(
            registry.get<orientation>(legs.first),
            registry.get<orientation>(legs.second));

        *std::array{&entities.knee_left_hinge, &entities.knee_right_hinge}[i] = hinge_ent;
    }

    /* Ankles */
    for (auto i = 0; i < 2; ++i) {
        auto ankle = std::array{
            std::make_pair(entities.leg_lower_left, entities.foot_left),
            std::make_pair(entities.leg_lower_right, entities.foot_right)
        }[i];

        auto con_entity = registry.create();
        auto cone_rot = quaternion_axis_angle({1, 0, 0}, to_radians(5));

        auto &cone_con = make_constraint<cone_constraint>(con_entity, registry, ankle.first, ankle.second);
        cone_con.pivot[0] = {0, -rag_def.leg_lower_size.y / 2, 0};
        cone_con.pivot[1] = {0, -rag_def.foot_size.y, 0};
        cone_con.frame = matrix3x3_columns(
            rotate(cone_rot, -vector3_y),
            rotate(cone_rot, -vector3_x),
            rotate(cone_rot, vector3_z));
        cone_con.span_tan[0] = std::tan(to_radians(24));
        cone_con.span_tan[1] = std::tan(to_radians(50));
        cone_con.bump_stop_stiffness = 3000;
        cone_con.bump_stop_length = 0.03;

        auto &cvjoint = make_constraint<cvjoint_constraint>(con_entity, registry, ankle.first, ankle.second);
        cvjoint.pivot[0] = cone_con.pivot[0];
        cvjoint.pivot[1] = {0, rag_def.foot_size.y / 2, rag_def.leg_lower_size.z / 2};
        cvjoint.frame[0] = matrix3x3_columns(vector3_y, vector3_x, -vector3_z);
        cvjoint.frame[1] = matrix3x3_columns(vector3_y, vector3_x, -vector3_z);
        cvjoint.reset_angle(
            registry.get<orientation>(ankle.first),
            registry.get<orientation>(ankle.second));
        cvjoint.bend_friction_torque = to_Nm_per_radian(0.005);
        cvjoint.bend_damping = to_Nm_per_radian(0.05);

        *std::array{
            &entities.ankle_left_constraint,
            &entities.ankle_right_constraint
        }[i] = con_entity;
    }

    /* Upper torso-shoulders */
    for (auto i = 0; i < 2; ++i) {
        auto shoulder = std::array{entities.shoulder_left, entities.shoulder_right}[i];
        scalar side = to_sign(i == 0);
        auto con_entity = registry.create();

        auto cone_rot =
            quaternion_axis_angle({0, 0, 1}, to_radians(15 * side)) *
            quaternion_axis_angle({0, 1, 0}, to_radians(15 * side));
        auto shoulder_size_x = rag_def.torso_upper_size.x * scalar(0.352);
        auto shoulder_pos_x = rag_def.torso_upper_size.x / 2 * scalar(0.65);

        auto &cone_con = make_constraint<cone_constraint>(con_entity, registry, entities.torso_upper, shoulder);
        cone_con.pivot[0] = {(shoulder_pos_x - shoulder_size_x / 2) * side,
                             rag_def.torso_upper_size.y / 2 - rag_def.arm_upper_size.y / 2, 0};
        cone_con.pivot[1] = {shoulder_size_x, 0, 0};
        cone_con.frame = matrix3x3_columns(
            rotate(cone_rot, side * vector3_x),
            rotate(cone_rot, side * vector3_y),
            rotate(cone_rot, vector3_z));
        cone_con.span_tan[0] = std::tan(to_radians(30));
        cone_con.span_tan[1] = std::tan(to_radians(40));
        cone_con.bump_stop_stiffness = 3000;
        cone_con.bump_stop_length = 0.03;

        auto &cvjoint = make_constraint<cvjoint_constraint>(con_entity, registry, entities.torso_upper, shoulder);
        cvjoint.pivot[0] = cone_con.pivot[0];
        cvjoint.pivot[1] = {-shoulder_size_x / 2, 0, 0};
        cvjoint.frame[0] = matrix3x3_columns(side * vector3_x, side * vector3_y, vector3_z);
        cvjoint.frame[1] = matrix3x3_identity;
        cvjoint.twist_min = to_radians(-5);
        cvjoint.twist_max = -cvjoint.twist_min;
        cvjoint.reset_angle(
            registry.get<orientation>(entities.torso_upper),
            registry.get<orientation>(shoulder));
        cvjoint.twist_friction_torque = cvjoint.bend_friction_torque = to_Nm_per_radian(0.02);
        cvjoint.twist_damping = cvjoint.bend_damping = to_Nm_per_radian(0.2);
        cvjoint.twist_bump_stop_angle = to_radians(2);
        cvjoint.twist_bump_stop_stiffness = to_Nm_per_radian(5);

        *std::array{
            &entities.torso_upper_shoulder_left_constraint,
            &entities.torso_upper_shoulder_right_constraint
        }[i] = con_entity;
    }

    /* Shoulder-upper arm */
    for (auto i = 0; i < 2; ++i) {
        auto shoulder = std::array{entities.shoulder_left, entities.shoulder_right}[i];
        auto arm = std::array{entities.arm_upper_left, entities.arm_upper_right}[i];
        scalar side = to_sign(i == 0);
        auto con_entity = registry.create();
        auto cone_rot = quaternion_axis_angle(normalize(vector3{0, 1, -side}), to_radians(45));

        auto &cone_con = make_constraint<cone_constraint>(con_entity, registry, shoulder, arm);
        cone_con.pivot[0] = {shoulder_size.x / 2, 0, 0};
        cone_con.pivot[1] = {rag_def.arm_upper_size.x, 0, 0};
        cone_con.frame = matrix3x3_columns(
            rotate(cone_rot, vector3_x),
            rotate(cone_rot, vector3_y),
            rotate(cone_rot, vector3_z));
        cone_con.span_tan[0] = std::tan(to_radians(45));
        cone_con.span_tan[1] = std::tan(to_radians(45));
        cone_con.bump_stop_stiffness = 3000;
        cone_con.bump_stop_length = 0.03;

        auto &cvjoint = make_constraint<cvjoint_constraint>(con_entity, registry, shoulder, arm);
        cvjoint.pivot[0] = cone_con.pivot[0];
        cvjoint.pivot[1] = {-rag_def.arm_upper_size.x / 2, 0, 0};
        cvjoint.frame[0] = matrix3x3_identity;
        cvjoint.frame[1] = matrix3x3_identity;
        cvjoint.twist_min = to_radians(-45);
        cvjoint.twist_max = -cvjoint.twist_min;
        cvjoint.reset_angle(
            registry.get<orientation>(shoulder),
            registry.get<orientation>(arm));
        cvjoint.twist_friction_torque = cvjoint.bend_friction_torque = to_Nm_per_radian(0.02);
        cvjoint.twist_damping = cvjoint.bend_damping = to_Nm_per_radian(0.2);
        cvjoint.twist_bump_stop_angle = to_radians(4);
        cvjoint.twist_bump_stop_stiffness = to_Nm_per_radian(5);

        *std::array{
            &entities.shoulder_arm_upper_left_constraint,
            &entities.shoulder_arm_upper_right_constraint
        }[i] = con_entity;
    }

    /* Elbows */
    for (auto i = 0; i < 2; ++i) {
        auto arm_upper = std::array{entities.arm_upper_left, entities.arm_upper_right}[i];
        auto arm_lower = std::array{entities.arm_lower_left, entities.arm_lower_right}[i];
        auto con_entity = registry.create();

        auto &hinge = make_constraint<hinge_constraint>(con_entity, registry, arm_upper, arm_lower);
        hinge.pivot[0] = {rag_def.arm_upper_size.x / 2, 0, 0};
        hinge.pivot[1] = {-rag_def.arm_lower_size.x / 2, 0, 0};
        hinge.set_axes({0, 1, 0}, {0, 1, 0});
        hinge.angle_min = 0;
        hinge.angle_max = to_radians(140);
        hinge.damping = 0.1;
        hinge.friction_torque = 0.02;
        hinge.bump_stop_angle = to_radians(10);
        hinge.bump_stop_stiffness = to_Nm_per_radian(5);
        hinge.reset_angle(
            registry.get<orientation>(arm_upper),
            registry.get<orientation>(arm_lower));

        *std::array{&entities.elbow_left_hinge, &entities.elbow_right_hinge}[i] = con_entity;
    }

    /* Forearm twist */
    for (auto i = 0; i < 2; ++i) {
        auto arm = std::array{entities.arm_lower_left, entities.arm_lower_right}[i];
        auto twist = std::array{entities.arm_twist_left, entities.arm_twist_right}[i];

        auto [hinge_ent, hinge] = make_constraint<hinge_constraint>(registry, arm, twist);
        hinge.pivot[0] = {0, 0, 0};
        hinge.pivot[1] = {0, 0, 0};
        hinge.set_axes({1, 0, 0}, {1, 0, 0});
        hinge.angle_min = -pi_half;
        hinge.angle_max = pi_half;
        hinge.damping = 0.1;
        hinge.friction_torque = 0.02;
        hinge.bump_stop_angle = to_radians(10);
        hinge.bump_stop_stiffness = to_Nm_per_radian(5);
        hinge.reset_angle(
            registry.get<orientation>(arm),
            registry.get<orientation>(twist));

        *std::array{&entities.arm_twist_left_hinge, &entities.arm_twist_right_hinge}[i] = hinge_ent;
    }
    /* Forearm-hand */
    for (auto i = 0; i < 2; ++i) {
        auto twist = std::array{entities.arm_twist_left, entities.arm_twist_right}[i];
        auto hand = std::array{entities.hand_left, entities.hand_right}[i];
        auto con_entity = registry.create();

        auto &cone_con = make_constraint<cone_constraint>(con_entity, registry, twist, hand);
        cone_con.pivot[0] = {rag_def.arm_lower_size.x / 2, 0, 0};
        cone_con.pivot[1] = {rag_def.hand_size.x, 0, 0};
        cone_con.frame = matrix3x3_identity;
        cone_con.span_tan[0] = std::tan(to_radians(80));
        cone_con.span_tan[1] = std::tan(to_radians(30));
        cone_con.bump_stop_stiffness = 2000;
        cone_con.bump_stop_length = 0.03;

        auto &cvjoint = make_constraint<cvjoint_constraint>(con_entity, registry, twist, hand);
        cvjoint.pivot[0] = cone_con.pivot[0];
        cvjoint.pivot[1] = {-rag_def.hand_size.x / 2,
                            (rag_def.hand_size.y - rag_def.arm_lower_size.y) / 2,
                            (rag_def.hand_size.z - rag_def.arm_lower_size.z) / 2};
        cvjoint.frame[0] = matrix3x3_identity;
        cvjoint.frame[1] = matrix3x3_identity;
        cvjoint.reset_angle(
            registry.get<orientation>(twist),
            registry.get<orientation>(hand));
        cvjoint.bend_friction_torque = to_Nm_per_radian(0.004);
        cvjoint.bend_damping = to_Nm_per_radian(0.02);

        *std::array{&entities.wrist_left_constraint, &entities.wrist_right_constraint}[i] = con_entity;
    }

    return entities;
}

}