#ifndef EDYN_CONSTRAINTS_SPRINGDAMPER_CONSTRAINT_HPP
#define EDYN_CONSTRAINTS_SPRINGDAMPER_CONSTRAINT_HPP

#include <array>
#include <vector>
#include "edyn/math/vector3.hpp"
#include "edyn/constraints/constraint_body.hpp"
#include "edyn/constraints/constraint_base.hpp"

namespace edyn {

struct constraint_row_prep_cache;

struct springdamper_constraint : public constraint_base {
    vector3 m_pivotA;
    vector3 m_ctrl_arm_pivotA;
    vector3 m_ctrl_arm_pivotB;
    vector3 m_ctrl_arm_pivot;

    scalar m_spring_stiffness;
    scalar m_spring_rest_length;
    scalar m_spring_min_length;
    scalar m_spring_offset;

    scalar m_second_spring_stiffness;
    scalar m_second_spring_rest_length;
    scalar m_second_spring_min_length;

    scalar m_spring_divider_length;

    scalar m_bumpstop_stiffness;
    scalar m_bumpstop_rest_length;
    scalar m_bumpstop_offset;

    scalar m_piston_rod_length;
    scalar m_damper_body_length;
    scalar m_damper_body_offset;

    scalar m_spring_perch_offset;
    scalar m_slow_compression_damping;
    scalar m_slow_rebound_damping;
    scalar m_fast_compression_damping;
    scalar m_fast_rebound_damping;
    scalar m_compression_knee_speed;
    scalar m_rebound_knee_speed;

    scalar m_damping_ratio;
    vector3 m_spring_damper_dir;

    scalar get_spring_deflection(entt::registry &) const;
    scalar get_preload() const;
    scalar get_combined_spring_stiffness() const;
    vector3 get_world_ctrl_arm_pivot(entt::registry &) const;
    scalar get_damping_force(scalar speed) const;

    struct {
        scalar spring {};
        scalar bumpstop {};
        scalar damper {};
        scalar damper_limit {};
    } applied_impulse {};

    void prepare(
        const entt::registry &, entt::entity,
        constraint_row_prep_cache &cache, scalar dt,
        const constraint_body &bodyA, const constraint_body &bodyB);

    void store_applied_impulses(const std::vector<scalar> &impulses);
};

template<typename Archive>
void serialize(Archive &archive, springdamper_constraint &con) {
    archive(con.body);
    archive(con.m_pivotA);
    archive(con.m_ctrl_arm_pivotA);
    archive(con.m_ctrl_arm_pivotB);
    archive(con.m_ctrl_arm_pivot);
    archive(con.m_spring_stiffness);
    archive(con.m_spring_rest_length);
    archive(con.m_spring_min_length);
    archive(con.m_spring_offset);
    archive(con.m_second_spring_stiffness);
    archive(con.m_second_spring_rest_length);
    archive(con.m_second_spring_min_length);
    archive(con.m_spring_divider_length);
    archive(con.m_bumpstop_stiffness);
    archive(con.m_bumpstop_rest_length);
    archive(con.m_bumpstop_offset);
    archive(con.m_piston_rod_length);
    archive(con.m_damper_body_length);
    archive(con.m_damper_body_offset);
    archive(con.m_spring_perch_offset);
    archive(con.m_slow_compression_damping);
    archive(con.m_slow_rebound_damping);
    archive(con.m_fast_compression_damping);
    archive(con.m_fast_rebound_damping);
    archive(con.m_compression_knee_speed);
    archive(con.m_rebound_knee_speed);
    archive(con.m_damping_ratio);
    archive(con.m_spring_damper_dir);
}

}

#endif // EDYN_CONSTRAINTS_SPRINGDAMPER_CONSTRAINT_HPP
