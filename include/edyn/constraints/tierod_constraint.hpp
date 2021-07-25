#ifndef EDYN_CONSTRAINTS_TIEROD_CONSTRAINT_HPP
#define EDYN_CONSTRAINTS_TIEROD_CONSTRAINT_HPP

#include <entt/fwd.hpp>
#include "constraint_base.hpp"
#include "edyn/math/vector3.hpp"
#include "edyn/constraints/prepare_constraints.hpp"

namespace edyn {

struct tierod_constraint : public constraint_base {
    vector3 pivotA;
    vector3 pivotB;
    vector3 pivotA_offset {vector3_zero};
    scalar rod_length;

    // upper control arm pivots
    vector3 upper_pivotA;
    vector3 upper_pivotB;

    // lower control arm pivots
    vector3 lower_pivotA;
    vector3 lower_pivotB;

    // length of control arms
    scalar upper_length;
    scalar lower_length;

    // Steering axis in wheel object space.
    vector3 steering_axis;

    // Vector from steering axis to pivot1
    vector3 steering_arm;
    scalar steering_arm_length;
    scalar steering_arm_angle;

    void update_steering_axis();
    void update_steering_arm();
};

template<>
void prepare_constraints<tierod_constraint>(entt::registry &, row_cache &, scalar dt);

}

#endif // EDYN_CONSTRAINTS_TIEROD_CONSTRAINT_HPP