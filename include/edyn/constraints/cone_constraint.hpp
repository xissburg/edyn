#ifndef EDYN_CONSTRAINTS_CONE_CONSTRAINT_HPP
#define EDYN_CONSTRAINTS_CONE_CONSTRAINT_HPP

#include "edyn/constraints/constraint_base.hpp"
#include "edyn/math/matrix3x3.hpp"
#include "edyn/math/vector3.hpp"
#include "edyn/util/array_util.hpp"

namespace edyn {

struct constraint_row_prep_cache;

/**
 * @brief Constrains a point in one body to a cone in the other body.
 */
struct cone_constraint : public constraint_base {
    // Pivots in object space. The first pivot is the location of the cone
    // vertex in the object space of the first body. The second pivot is
    // the point in the object space of the second body to be restricted to
    // the cone volume.
    std::array<vector3, 2> pivot;

    // Frame in the first body's object space. This is an orthonormal basis
    // where the first column of the matrix is the direction of the cone.
    matrix3x3 frame{matrix3x3_identity};

    // Tangent of the span angle along the y and z axes of the first body's
    // frame.
    std::array<scalar, 2> span_tan;

    // Restitution of the cone limits, i.e. how much the body will bounce
    // once it hits the surface of the cone.
    scalar restitution{};

    // Bump stop spring stiffness.
    scalar bump_stop_stiffness{};

    // Size of bump stop. When the distance between the second body's pivot to
    // the cone surface gets below this value, the bump stop spring will start
    // to be effective.
    scalar bump_stop_length{};

    static constexpr auto num_rows = 2;
    std::array<scalar, num_rows> impulse {make_array<num_rows>(scalar{})};

    void prepare(
        const entt::registry &, entt::entity,
        constraint_row_prep_cache &cache, scalar dt,
        const vector3 &originA, const vector3 &posA, const quaternion &ornA,
        const vector3 &linvelA, const vector3 &angvelA,
        scalar inv_mA, const matrix3x3 &inv_IA,
        const vector3 &originB, const vector3 &posB, const quaternion &ornB,
        const vector3 &linvelB, const vector3 &angvelB,
        scalar inv_mB, const matrix3x3 &inv_IB);
};

template<typename Archive>
void serialize(Archive &archive, cone_constraint &c) {
    archive(c.body, c.pivot, c.frame);
    archive(c.span_tan, c.restitution);
    archive(c.bump_stop_stiffness, c.bump_stop_length);
    archive(c.impulse);
};

}

#endif // EDYN_CONSTRAINTS_CONE_CONSTRAINT_HPP
