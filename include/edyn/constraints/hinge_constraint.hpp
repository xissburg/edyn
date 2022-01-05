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
    std::array<matrix3x3, 2> frame;

    // Angular limits. `angle_min` must be smaller than `angle_max`.
    scalar angle_min{}, angle_max{};

    // Angular limit restitution.
    scalar limit_restitution{};

    scalar friction_torque{};

    std::array<scalar, 6> impulse {make_array<6>(scalar{})};

    /**
     * @brief Set hinge axes.
     * @param axisA Axis in the first rigid body, in object space.
     * @param axisB Axis in the second rigid body, in object space.
     */
    void set_axes(const vector3 &axisA, const vector3 &axisB);
};

template<>
void prepare_constraints<hinge_constraint>(entt::registry &, row_cache &, scalar dt);


template<>
bool solve_position_constraints<hinge_constraint>(entt::registry &, scalar dt);

}

#endif // EDYN_CONSTRAINTS_HINGE_CONSTRAINT_HPP
