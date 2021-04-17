#ifndef EDYN_CONSTRAINTS_HINGE_CONSTRAINT_HPP
#define EDYN_CONSTRAINTS_HINGE_CONSTRAINT_HPP

#include <array>
#include <entt/fwd.hpp>
#include "edyn/math/matrix3x3.hpp"
#include "edyn/constraints/constraint_base.hpp"
#include "edyn/constraints/prepare_constraints.hpp"

namespace edyn {

struct hinge_constraint : public constraint_base {
    std::array<vector3, 2> pivot;
    std::array<matrix3x3, 2> frame;

    void set_axis(const quaternion &ornA,
                  const vector3 &axisA, const vector3 &axisB);
};

template<>
void prepare_constraints<hinge_constraint>(entt::registry &, row_cache &, scalar dt);

}

#endif // EDYN_CONSTRAINTS_HINGE_CONSTRAINT_HPP
