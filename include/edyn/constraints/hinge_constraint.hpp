#ifndef EDYN_CONSTRAINTS_HINGE_CONSTRAINT_HPP
#define EDYN_CONSTRAINTS_HINGE_CONSTRAINT_HPP

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
struct quaternion;

/**
 * @brief Constrains a pair of rigid bodies to have a pivot point match in space
 * plus only allows rotation to happen along a specified axis.
 */
struct hinge_constraint : public constraint_base {
    // Pivots in object space.
    std::array<vector3, 2> pivot;

    // Frames in object space. The first column of the matrix is the hinge axis.
    std::array<matrix3x3, 2> frame{matrix3x3_identity, matrix3x3_identity};

    // Angular limits. `angle_min` must be smaller than `angle_max`.
    scalar angle_min{}, angle_max{};

    // Angular limit restitution.
    scalar limit_restitution{};

    // When limits are set, if the angle between the two frames along the hinge
    // axis is smaller than `angle_min + bump_stop_angle` or greater than
    // `angle_max - bump_stop_angle` a force proportional to the proximity to
    // the angular limit multiplied by `bump_stop_stiffness` will be applied.
    scalar bump_stop_angle{};

    // Spring rate of bump stop in Nm/rad.
    scalar bump_stop_stiffness{};

    // Torque applied in the opposite direction of the angular velocity along
    // the hinge axis, simulating resistance to motion in the hinge.
    scalar torque{};
    scalar speed{};

    // Relative angle between the two frames along the hinge axis that the
    // spring force should try to maintain.
    scalar rest_angle{};

    // Spring stiffness in Nm/rad.
    scalar stiffness{};

    // Damping rate in Nm/(rad/s).
    scalar damping{};

    // Current relative angle between the two frames along the hinge axis.
    // Do not modify.
    scalar angle{};

    struct {
        std::array<scalar, 3> linear {};
        std::array<scalar, 2> hinge {};
        scalar limit {};
        scalar bump_stop {};
        scalar spring {};
        scalar torque {};
    } applied_impulse {};

    /**
     * @brief Set hinge axes.
     * @param axisA Axis in the first rigid body, in object space.
     * @param axisB Axis in the second rigid body, in object space.
     */
    void set_axes(const vector3 &axisA, const vector3 &axisB);

    /**
     * @brief Recalculates the current angle. Should be called after changing
     * the constraint frames so that the relative angle is set correctly.
     * @param ornA Orientation of the first rigid body.
     * @param ornB Orientation of the second rigid body.
     */
    void reset_angle(const quaternion &ornA, const quaternion &ornB);

    void prepare(
        const entt::registry &, entt::entity,
        constraint_row_prep_cache &cache, scalar dt,
        const constraint_body &bodyA, const constraint_body &bodyB);

    void solve_position(position_solver &solver);

    void store_applied_impulses(const std::vector<scalar> &impulses);
};

template<typename Archive>
void serialize(Archive &archive, hinge_constraint &c) {
    archive(c.body);
    archive(c.pivot);
    archive(c.frame);
    archive(c.angle_min, c.angle_max, c.limit_restitution);
    archive(c.bump_stop_angle, c.bump_stop_stiffness);
    archive(c.torque, c.speed);
    archive(c.rest_angle, c.stiffness, c.damping);
    archive(c.angle);
    archive(c.applied_impulse.linear);
    archive(c.applied_impulse.hinge);
    archive(c.applied_impulse.limit);
    archive(c.applied_impulse.bump_stop);
    archive(c.applied_impulse.spring);
    archive(c.applied_impulse.torque);
}

}

#endif // EDYN_CONSTRAINTS_HINGE_CONSTRAINT_HPP
