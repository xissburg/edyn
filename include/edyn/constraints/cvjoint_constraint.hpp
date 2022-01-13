#ifndef EDYN_CONSTRAINTS_CVJOINT_CONSTRAINT_HPP
#define EDYN_CONSTRAINTS_CVJOINT_CONSTRAINT_HPP

#include <array>
#include <entt/entity/fwd.hpp>
#include "edyn/math/vector3.hpp"
#include "edyn/math/matrix3x3.hpp"
#include "edyn/constraints/constraint_base.hpp"
#include "edyn/constraints/prepare_constraints.hpp"
#include "edyn/util/array.hpp"

namespace edyn {

/**
 * @brief Constant-velocity joint.
 */
struct cvjoint_constraint : public constraint_base {
    // Pivots in object space.
    std::array<vector3, 2> pivot;

    // Frames in object space. The first column of the matrix is the spin axis.
    // The other axes are used to calculate the relative angle between frames.
    std::array<matrix3x3, 2> frame{matrix3x3_identity, matrix3x3_identity};

    // Angular limits. `angle_min` must be smaller than `angle_max`.
    scalar angle_min{}, angle_max{};

    // Angular limit restitution.
    scalar limit_restitution{};

    // Current relative angle between the two frames along the spin axis.
    // Do not modify.
    scalar angle{};

    static constexpr auto num_rows = 4;
    std::array<scalar, num_rows> impulse {make_array<num_rows>(scalar{})};

    /**
     * @brief Recalculates the current angle. Should be called after changing
     * the constraint frames so that the relative angle is set correctly.
     * @param ornA Orientation of the first rigid body.
     * @param ornB Orientation of the second rigid body.
     */
    void reset_angle(const quaternion &ornA, const quaternion &ornB);

    scalar relative_angle(const quaternion &ornA, const quaternion &ornB) const;
    scalar relative_angle(const quaternion &ornA, const quaternion &ornB,
                          const vector3 &spin_axisA, const vector3 &spin_axisB) const;
    void update_angle(scalar new_angle);
};

template<>
void prepare_constraints<cvjoint_constraint>(entt::registry &, row_cache &, scalar dt);

template<>
bool solve_position_constraints<cvjoint_constraint>(entt::registry &, scalar dt);

template<typename Archive>
void serialize(Archive &archive, cvjoint_constraint &c) {
    archive(c.body, c.pivot, c.frame);
    archive(c.angle_min, c.angle_max, c.limit_restitution);
};

}

#endif // EDYN_CONSTRAINTS_CVJOINT_CONSTRAINT_HPP
