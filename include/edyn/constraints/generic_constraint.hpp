#ifndef EDYN_CONSTRAINTS_GENERIC_CONSTRAINT_HPP
#define EDYN_CONSTRAINTS_GENERIC_CONSTRAINT_HPP

#include <array>
#include <entt/entity/fwd.hpp>
#include "edyn/math/matrix3x3.hpp"
#include "edyn/math/vector3.hpp"
#include "edyn/constraints/constraint_base.hpp"
#include "edyn/constraints/prepare_constraints.hpp"
#include "edyn/util/array.hpp"

namespace edyn {

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
    };

    std::array<vector3, 2> pivot;
    std::array<matrix3x3, 2> frame{matrix3x3_identity, matrix3x3_identity};
    std::array<linear_dof, 3> linear_dofs;
    std::array<angular_dof, 3> angular_dofs;

    static constexpr auto num_rows = 24;
    std::array<scalar, num_rows> impulse {make_array<num_rows, scalar>(0)};
};

template<>
void prepare_constraints<generic_constraint>(entt::registry &, row_cache &, scalar dt);

template<>
bool solve_position_constraints<generic_constraint>(entt::registry &, scalar dt);

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
    archive(c.impulse);
}

}

#endif // EDYN_CONSTRAINTS_GENERIC_CONSTRAINT_HPP
