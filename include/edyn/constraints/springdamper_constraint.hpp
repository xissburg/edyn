#ifndef EDYN_CONSTRAINTS_SPRINGDAMPER_CONSTRAINT_HPP
#define EDYN_CONSTRAINTS_SPRINGDAMPER_CONSTRAINT_HPP

#include "edyn/math/vector3.hpp"
#include "edyn/math/linear_curve.hpp"
#include "edyn/constraints/constraint_base.hpp"
#include "edyn/constraints/prepare_constraints.hpp"
#include "edyn/util/array.hpp"

namespace edyn {

struct springdamper_constraint : public constraint_base {
    linear_curve m_stiffness_curve;

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
    scalar m_inclination_factor;

    void set_constant_spring_stiffness();
    void set_constant_spring_stiffness(scalar stiffness, scalar max_defl);

    void set_dual_spring_stiffness();
    void set_dual_spring_stiffness(scalar primary_stiffness, scalar primary_max_defl,
                                   scalar secondary_stiffness, scalar secondary_max_defl);

    scalar get_spring_deflection(entt::registry &) const;
    scalar get_preload() const;
    scalar get_combined_spring_stiffness() const;
    vector3 get_world_ctrl_arm_pivot(entt::registry &) const;
    scalar get_damping_force(scalar speed) const;

    static const auto num_rows = 3;
    std::array<scalar, num_rows> impulse = make_array<num_rows>(scalar{});
};

template<>
void prepare_constraints<springdamper_constraint>(entt::registry &, row_cache &, scalar dt);

}

#endif // EDYN_CONSTRAINTS_SPRINGDAMPER_CONSTRAINT_HPP
