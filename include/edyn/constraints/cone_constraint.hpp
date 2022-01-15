#ifndef EDYN_CONSTRAINTS_CONE_CONSTRAINT_HPP
#define EDYN_CONSTRAINTS_CONE_CONSTRAINT_HPP

#include "edyn/constraints/constraint_base.hpp"
#include "edyn/constraints/prepare_constraints.hpp"
#include "edyn/math/matrix3x3.hpp"
#include "edyn/math/vector3.hpp"
#include "edyn/util/array.hpp"

namespace edyn {

struct cone_constraint : public constraint_base {
    // Pivots in object space.
    std::array<vector3, 2> pivot;

    // Frames in object space. The first column of the matrix is the direction
    // of the cone.
    std::array<matrix3x3, 2> frame{matrix3x3_identity, matrix3x3_identity};

    std::array<scalar, 2> span;

    scalar limit_restitution{};

    scalar impulse{};
};

template<>
void prepare_constraints<cone_constraint>(entt::registry &, row_cache &, scalar dt);

template<>
bool solve_position_constraints<cone_constraint>(entt::registry &, scalar dt);

template<typename Archive>
void serialize(Archive &archive, cone_constraint &c) {
    archive(c.body, c.pivot, c.span);
};

}

#endif // EDYN_CONSTRAINTS_CONE_CONSTRAINT_HPP
