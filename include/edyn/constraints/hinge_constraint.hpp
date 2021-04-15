#ifndef EDYN_CONSTRAINTS_HINGE_CONSTRAINT_HPP
#define EDYN_CONSTRAINTS_HINGE_CONSTRAINT_HPP

#include <array>
#include <entt/fwd.hpp>
#include "edyn/math/matrix3x3.hpp"

namespace edyn {

class row_cache;
struct constraint;

struct hinge_constraint {
    std::array<vector3, 2> pivot;
    std::array<matrix3x3, 2> frame;

    void set_axis(const quaternion &ornA,
                  const vector3 &axisA, const vector3 &axisB);
    void prepare(entt::entity, const constraint &, entt::registry &, row_cache &cache, scalar dt);
};

}

#endif // EDYN_CONSTRAINTS_HINGE_CONSTRAINT_HPP