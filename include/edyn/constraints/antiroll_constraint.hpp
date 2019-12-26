#ifndef EDYN_CONSTRAINTS_ANTIROLL_CONSTRAINT_HPP
#define EDYN_CONSTRAINTS_ANTIROLL_CONSTRAINT_HPP

#include <entt/fwd.hpp>
#include "constraint_base.hpp"
#include "edyn/math/vector3.hpp"

namespace edyn {

struct antiroll_constraint : public constraint_base<antiroll_constraint> {
    entt::entity third_entity;

    // Which side this antiroll constraint connects to: 1 left, -1 right.
    scalar side;

    scalar stiffness; // torsional stiffness in N/m/deg (netwon-meters per degrees)
    vector3 pivotA; // point where it attaches to the chassis

    vector3 ctrl_arm_pivotA; // pivot on the chassis of the control arm to which the antiroll bar is connected
    vector3 ctrl_arm_pivotB; // pivot on the wheel of the control arm to which the antiroll bar is connected
    vector3 ctrl_arm_pivot; // point where it attaches to the control arm, in control arm space

    // Same as above for the control arm on the other side
    vector3 other_ctrl_arm_pivotA; 
    vector3 other_ctrl_arm_pivotC; 
    vector3 other_ctrl_arm_pivot;

    scalar angle;

    void init(entt::entity, constraint &, const relation &, entt::registry &);
    void prepare(entt::entity, constraint &, const relation &, entt::registry &, scalar dt);
};

}

#endif // EDYN_CONSTRAINTS_ANTIROLL_CONSTRAINT_HPP