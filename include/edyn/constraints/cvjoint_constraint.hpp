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
    std::array<matrix3x3, 2> frame{matrix3x3_identity, matrix3x3_identity};
    std::array<vector3, 2> pivot;

    static constexpr auto num_rows = 4;
    std::array<scalar, num_rows> impulse {make_array<num_rows>(scalar{})};
};

template<>
void prepare_constraints<cvjoint_constraint>(entt::registry &, row_cache &, scalar dt);

template<>
bool solve_position_constraints<cvjoint_constraint>(entt::registry &, scalar dt);

template<typename Archive>
void serialize(Archive &archive, cvjoint_constraint &c) {
    archive(c.body, c.frame, c.pivot);
};

}

#endif // EDYN_CONSTRAINTS_CVJOINT_CONSTRAINT_HPP
