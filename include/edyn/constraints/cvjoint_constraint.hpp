#ifndef EDYN_CONSTRAINTS_CVJOINT_CONSTRAINT_HPP
#define EDYN_CONSTRAINTS_CVJOINT_CONSTRAINT_HPP

#include <array>
#include <vector>
#include <entt/entity/fwd.hpp>
#include "edyn/math/vector3.hpp"
#include "edyn/math/matrix3x3.hpp"
#include "edyn/constraints/constraint_base.hpp"
#include "edyn/constraints/constraint_body.hpp"

namespace edyn {

struct constraint_row_prep_cache;
class position_solver;

/**
 * @brief Constant-velocity joint. It constrains the angle and angular
 * velocity to an independent axis on each body.
 */
struct cvjoint_constraint : public constraint_base {
    // Pivots in object space.
    std::array<vector3, 2> pivot;

    // Frames in object space. A frame is an orthonormal basis. The first
    // column of the matrix is the twist axis. The other axes are used to
    // calculate the relative angle between frames.
    std::array<matrix3x3, 2> frame{matrix3x3_identity, matrix3x3_identity};

    // Twist angular limits, in radians. `twist_min` must be smaller than
    // `twist_max`. The twist angle is calculated by rotating body B's frame
    // onto A's space so that their x axes coincide, and then the angle
    // between the y axis of the transformed B's frame with respect to the A's
    // y axis is calculated.
    scalar twist_min{}, twist_max{};

    // Angular limit restitution.
    scalar twist_restitution{};

    // Current relative angle between the two frames along the twist axis.
    // Do not modify.
    scalar twist_angle{};

    // When limits are set, if the angle between the two frames along the twist
    // axis is smaller than `twist_min + twist_bump_stop_angle` or greater than
    // `twist_max - twist_bump_stop_angle` a force proportional to the proximity
    // to the angular limit multiplied by `twist_bump_stop_stiffness` will be
    // applied.
    scalar twist_bump_stop_angle{};

    // Spring rate of bump stop for twisting in Nm/rad.
    scalar twist_bump_stop_stiffness{};

    // Torque applied in the opposite direction of the angular velocity along
    // the twist axis.
    scalar twist_friction_torque{};

    // Relative angle between the two frames along the twist axis that the
    // spring force should try to maintain.
    scalar twist_rest_angle{};

    // Spring stiffness for twisting in Nm/rad.
    scalar twist_stiffness{};

    // Damping rate for twisting in Nm/(rad/s).
    scalar twist_damping{};

    // Direction CV-joint tries to point towards, specified in the object space
    // of body A.
    vector3 rest_direction{};

    // Stiffness of spring that forces the twist axis of body B to align with
    // the frame of body A along `rest_direction`.
    scalar bend_stiffness{};

    // Torque applied in the opposite direction of the relative angular
    // velocity in the plane with the twist axis of body 0 as normal, thus
    // slowing down bending motion.
    scalar bend_friction_torque{};

    // Similar to `bend_friction_torque` but the force is proportional to
    // the relative angular velocity.
    scalar bend_damping{};

    struct {
        std::array<scalar, 3> linear {};
        scalar twist_limit {};
        scalar twist_bump_stop {};
        scalar twist_spring {};
        scalar twist_friction_damping {};
        scalar bend_friction_damping {};
        scalar bend_spring {};
    } applied_impulse {};

    /**
     * @brief Recalculates the current angle. Should be called after changing
     * the constraint frames so that the relative angle is set correctly.
     * @param ornA Orientation of the first rigid body.
     * @param ornB Orientation of the second rigid body.
     */
    void reset_angle(const quaternion &ornA, const quaternion &ornB);

    scalar relative_angle(const quaternion &ornA, const quaternion &ornB) const;
    scalar relative_angle(const quaternion &ornA, const quaternion &ornB,
                          const vector3 &twist_axisA, const vector3 &twist_axisB) const;
    void update_angle(scalar new_angle);

    void prepare(
        const entt::registry &, entt::entity,
        constraint_row_prep_cache &cache, scalar dt,
        const constraint_body &bodyA, const constraint_body &bodyB);

    void solve_position(position_solver &solver);

    void store_applied_impulses(const std::vector<scalar> &impulses);
};

template<typename Archive>
void serialize(Archive &archive, cvjoint_constraint &c) {
    archive(c.body, c.pivot, c.frame);
    archive(c.twist_min, c.twist_max);
    archive(c.twist_restitution);
    archive(c.twist_angle);
    archive(c.twist_bump_stop_angle);
    archive(c.twist_bump_stop_stiffness);
    archive(c.twist_friction_torque);
    archive(c.twist_rest_angle);
    archive(c.twist_stiffness);
    archive(c.twist_damping);
    archive(c.rest_direction);
    archive(c.bend_stiffness);
    archive(c.bend_friction_torque);
    archive(c.bend_damping);
    archive(c.applied_impulse.linear);
    archive(c.applied_impulse.twist_limit);
    archive(c.applied_impulse.twist_bump_stop);
    archive(c.applied_impulse.twist_spring);
    archive(c.applied_impulse.twist_friction_damping);
    archive(c.applied_impulse.bend_friction_damping);
    archive(c.applied_impulse.bend_spring);
};

}

#endif // EDYN_CONSTRAINTS_CVJOINT_CONSTRAINT_HPP
