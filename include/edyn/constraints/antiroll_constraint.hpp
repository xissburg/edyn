#ifndef EDYN_CONSTRAINTS_ANTIROLL_CONSTRAINT_HPP
#define EDYN_CONSTRAINTS_ANTIROLL_CONSTRAINT_HPP

#include <vector>
#include <entt/fwd.hpp>
#include "edyn/constraints/constraint_body.hpp"
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

    scalar applied_impulse;

    void prepare(
        const entt::registry &, entt::entity,
        constraint_row_prep_cache &cache, scalar dt,
        const constraint_body &bodyA, const constraint_body &bodyB, const constraint_body &bodyC);

    void store_applied_impulses(const std::vector<scalar> &impulses);
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
    archive(con.applied_impulse);
}

}

#endif // EDYN_CONSTRAINTS_ANTIROLL_CONSTRAINT_HPP
