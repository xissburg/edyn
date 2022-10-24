#ifndef EDYN_CONSTRAINTS_GENERIC_CONSTRAINT_HPP
#define EDYN_CONSTRAINTS_GENERIC_CONSTRAINT_HPP

#include <array>
#include <vector>
#include <entt/entity/fwd.hpp>
#include "edyn/math/matrix3x3.hpp"
#include "edyn/math/vector3.hpp"
#include "edyn/constraints/constraint_base.hpp"
#include "edyn/constraints/constraint_body.hpp"

namespace edyn {

struct constraint_row_prep_cache;
class position_solver;

struct generic_constraint : public constraint_base {
    struct linear_dof {
        bool limit_enabled {true};
        scalar offset_min{};
        scalar offset_max{};
        scalar limit_restitution{};
        scalar bump_stop_length{};
        scalar bump_stop_stiffness{};
        scalar friction_force{};
        scalar rest_offset{};
        scalar spring_stiffness{};
        scalar damping{};

        struct {
            scalar limit;
            scalar bump_stop;
            scalar spring;
            scalar friction_damping;
        } applied_impulse;
    };

    struct angular_dof {
        bool limit_enabled {true};
        scalar angle_min{};
        scalar angle_max{};
        scalar limit_restitution{};
        scalar bump_stop_angle{};
        scalar bump_stop_stiffness{};
        scalar friction_torque{};
        scalar rest_angle{};
        scalar spring_stiffness{};
        scalar damping{};
        scalar current_angle{};

        struct {
            scalar limit;
            scalar bump_stop;
            scalar spring;
            scalar friction_damping;
        } applied_impulse;
    };

    std::array<vector3, 2> pivot;
    std::array<matrix3x3, 2> frame{matrix3x3_identity, matrix3x3_identity};
    std::array<linear_dof, 3> linear_dofs;
    std::array<angular_dof, 3> angular_dofs;

    void prepare(
        const entt::registry &, entt::entity,
        constraint_row_prep_cache &cache, scalar dt,
        const constraint_body &bodyA, const constraint_body &bodyB);

    void solve_position(position_solver &solver);

    void store_applied_impulses(const std::vector<scalar> &impulses);
};

template<typename Archive>
void serialize(Archive &archive, generic_constraint::linear_dof &dof) {
    archive(dof.limit_enabled);
    archive(dof.offset_min, dof.offset_max);
    archive(dof.limit_restitution);
    archive(dof.bump_stop_length, dof.bump_stop_stiffness);
    archive(dof.friction_force);
    archive(dof.rest_offset);
    archive(dof.spring_stiffness, dof.damping);
}

template<typename Archive>
void serialize(Archive &archive, generic_constraint::angular_dof &dof) {
    archive(dof.limit_enabled);
    archive(dof.angle_min, dof.angle_max);
    archive(dof.limit_restitution);
    archive(dof.bump_stop_angle, dof.bump_stop_stiffness);
    archive(dof.friction_torque);
    archive(dof.rest_angle);
    archive(dof.spring_stiffness, dof.damping);
    archive(dof.current_angle);
}

template<typename Archive>
void serialize(Archive &archive, generic_constraint &c) {
    archive(c.body);
    archive(c.pivot);
    archive(c.frame);
    archive(c.linear_dofs);
    archive(c.angular_dofs);
}

}

#endif // EDYN_CONSTRAINTS_GENERIC_CONSTRAINT_HPP
