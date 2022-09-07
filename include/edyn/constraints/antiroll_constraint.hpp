#ifndef EDYN_CONSTRAINTS_ANTIROLL_CONSTRAINT_HPP
#define EDYN_CONSTRAINTS_ANTIROLL_CONSTRAINT_HPP

#include <entt/fwd.hpp>
#include "edyn/math/vector3.hpp"
#include "edyn/constraints/constraint_base.hpp"

namespace edyn {

struct constraint_row_prep_cache;
struct quaternion;
class position_solver;

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

    void prepare(
        const entt::registry &, entt::entity,
        constraint_row_prep_cache &cache, scalar dt,
        const vector3 &originA, const vector3 &posA, const quaternion &ornA,
        const vector3 &linvelA, const vector3 &angvelA,
        const vector3 &originB, const vector3 &posB, const quaternion &ornB,
        const vector3 &linvelB, const vector3 &angvelB,
        const vector3 &originC, const vector3 &posC, const quaternion &ornC,
        const vector3 &linvelC, const vector3 &angvelC);
};

template<typename Archive>
void serialize(Archive &archive, antiroll_constraint &con) {
    archive(con.body);
    archive(con.m_third_entity);
    archive(con.m_stiffness);
    archive(con.m_pivotA);
    archive(con.m_ctrl_arm_pivotA);
    archive(con.m_ctrl_arm_pivotB);
    archive(con.m_ctrl_arm_pivot);
    archive(con.m_other_ctrl_arm_pivotA);
    archive(con.m_other_ctrl_arm_pivotC);
    archive(con.m_other_ctrl_arm_pivot);
    archive(con.impulse);
}

}

#endif // EDYN_CONSTRAINTS_ANTIROLL_CONSTRAINT_HPP
