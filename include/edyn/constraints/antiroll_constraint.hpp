#ifndef EDYN_CONSTRAINTS_ANTIROLL_CONSTRAINT_HPP
#define EDYN_CONSTRAINTS_ANTIROLL_CONSTRAINT_HPP

#include <entt/fwd.hpp>
#include "edyn/math/vector3.hpp"
#include "edyn/constraints/constraint_base.hpp"
#include "edyn/constraints/prepare_constraints.hpp"

namespace edyn {

struct antiroll_constraint : public constraint_base {
    entt::entity m_third_entity;

    // Torsional stiffness in Nm/deg (netwon-meters per degrees)
    scalar m_stiffness;

    // Point where it attaches to the chassis.
    vector3 m_pivotA;

    // Control arm pivot on chassis.
    vector3 m_ctrl_arm_pivotA;

    // Control arm pivot on wheel.
    vector3 m_ctrl_arm_pivotB;

    // Point where it attaches to the control arm, in control arm space
    vector3 m_ctrl_arm_pivot;

    // Same as above for the control arm on the other side
    vector3 m_other_ctrl_arm_pivotA;
    vector3 m_other_ctrl_arm_pivotC;
    vector3 m_other_ctrl_arm_pivot;

    scalar impulse;
};

template<>
void prepare_constraints<antiroll_constraint>(entt::registry &, row_cache &, scalar dt);

}

#endif // EDYN_CONSTRAINTS_ANTIROLL_CONSTRAINT_HPP
