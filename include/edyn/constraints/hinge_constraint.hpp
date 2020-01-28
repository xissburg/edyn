#ifndef EDYN_CONSTRAINTS_HINGE_CONSTRAINT_HPP
#define EDYN_CONSTRAINTS_HINGE_CONSTRAINT_HPP

#include <array>
#include "constraint_base.hpp"
#include "edyn/math/matrix3x3.hpp"

namespace edyn {

struct hinge_constraint : public  constraint_base<hinge_constraint> {
    std::array<vector3, 2> pivot;
    std::array<matrix3x3, 2> frame;

    void set_axis(const quaternion &ornA,
                  const vector3 &axisA, const vector3 &axisB);
    void init(constraint &, const relation &, entt::registry &);
    void prepare(constraint &, const relation &, entt::registry &, scalar dt);
};

}

#endif // EDYN_CONSTRAINTS_HINGE_CONSTRAINT_HPP