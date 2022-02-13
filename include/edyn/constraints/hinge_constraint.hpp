#ifndef EDYN_CONSTRAINTS_HINGE_CONSTRAINT_HPP
#define EDYN_CONSTRAINTS_HINGE_CONSTRAINT_HPP

#include <array>
#include <entt/entity/fwd.hpp>
#include "edyn/math/vector3.hpp"
#include "edyn/math/matrix3x3.hpp"
#include "edyn/constraints/constraint_base.hpp"
#include "edyn/constraints/prepare_constraints.hpp"
#include "edyn/util/array.hpp"

namespace edyn {

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
    scalar friction_torque{};

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

    // Applied impulses.
    // 0 - 2: linear point-to-point impulses.
    // 3: First hinge impulse.
    // 4: Second hinge impulse.
    // 5: Limit impulse.
    // 6: Bump stop impulse.
    // 7: Spring impulse.
    // 8: Friction and damping.
    static constexpr auto num_rows = 9;
    std::array<scalar, num_rows> impulse {make_array<num_rows>(scalar{})};

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
};

template<>
void prepare_constraints<hinge_constraint>(entt::registry &, row_cache &, scalar dt);

template<>
bool solve_position_constraints<hinge_constraint>(entt::registry &, scalar dt);

template<typename Archive>
void serialize(Archive &archive, hinge_constraint &c) {
    archive(c.body);
    archive(c.pivot);
    archive(c.frame);
    archive(c.angle_min, c.angle_max, c.limit_restitution);
    archive(c.bump_stop_angle, c.bump_stop_stiffness);
    archive(c.friction_torque);
    archive(c.rest_angle, c.stiffness, c.damping);
    archive(c.angle);
    archive(c.impulse);
}

}

#endif // EDYN_CONSTRAINTS_HINGE_CONSTRAINT_HPP
