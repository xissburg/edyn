#include "edyn/constraints/generic_constraint.hpp"
#include "edyn/dynamics/position_solver.hpp"
#include "edyn/dynamics/row_cache.hpp"
#include "edyn/math/math.hpp"
#include "edyn/math/transform.hpp"
#include "edyn/util/constraint_util.hpp"

namespace edyn {

void generic_constraint::prepare(
    const entt::registry &, entt::entity,
    constraint_row_prep_cache &cache, scalar dt,
    const constraint_body &bodyA, const constraint_body &bodyB) {

    auto pivotA = to_world_space(pivot[0], bodyA.origin, bodyA.orn);
    auto pivotB = to_world_space(pivot[1], bodyB.origin, bodyB.orn);
    auto rA = pivotA - bodyA.pos;
    auto rB = pivotB - bodyB.pos;

    auto pivot_offset = pivotB - pivotA;

    // Linear.
    for (int i = 0; i < 3; ++i) {
        auto &dof = linear_dofs[i];

        auto axisA = rotate(bodyA.orn, frame[0].column(i));
        auto J = std::array<vector3, 4>{axisA, cross(rA, axisA), -axisA, -cross(rB, axisA)};

        auto non_zero_limit = dof.offset_min < dof.offset_max;
        auto offset_proj = dot(pivot_offset, axisA);

        if (dof.limit_enabled) {
            EDYN_ASSERT(!(dof.offset_min > dof.offset_max));

            auto &row = cache.add_row();
            row.J = J;
            row.impulse = dof.applied_impulse.limit;
            auto &options = cache.get_options();

            if (non_zero_limit) {
                auto limit_error = scalar{};
                auto mid_point = (dof.offset_min + dof.offset_max) / scalar(2);

                if (offset_proj < mid_point) {
                    limit_error = dof.offset_min - offset_proj;
                    row.lower_limit = -large_scalar;
                    row.upper_limit = 0;
                } else {
                    limit_error = dof.offset_max - offset_proj;
                    row.lower_limit = 0;
                    row.upper_limit = large_scalar;
                }

                // Only assign error if the limits haven't been violated. The
                // position constraints will fix angular limit errors later.
                if (offset_proj > dof.offset_min && offset_proj < dof.offset_max) {
                    options.error = limit_error / dt;
                }

                options.restitution = dof.limit_restitution;
                options.erp = 0.9;
            } else {
                row.lower_limit = -large_scalar;
                row.upper_limit = large_scalar;
            }
        }

        // Linear bump stops.
        if (dof.limit_enabled && non_zero_limit && dof.bump_stop_stiffness > 0 && dof.bump_stop_length > 0) {
            auto bump_stop_deflection = scalar{};
            auto bump_stop_min = dof.offset_min + dof.bump_stop_length;
            auto bump_stop_max = dof.offset_max - dof.bump_stop_length;

            if (offset_proj < bump_stop_min) {
                bump_stop_deflection = offset_proj - bump_stop_min;
            } else if (offset_proj > bump_stop_max) {
                bump_stop_deflection = offset_proj - bump_stop_max;
            }

            auto &row = cache.add_row();
            row.J = J;
            row.impulse = dof.applied_impulse.bump_stop;

            auto spring_force = dof.bump_stop_stiffness * bump_stop_deflection;
            auto spring_impulse = spring_force * dt;
            row.lower_limit = std::min(spring_impulse, scalar(0));
            row.upper_limit = std::max(scalar(0), spring_impulse);

            auto &options = cache.get_options();
            options.error = -bump_stop_deflection / dt;
        }

        // Linear spring.
        if (dof.spring_stiffness > 0) {
            auto &row = cache.add_row();
            row.J = J;
            row.impulse = dof.applied_impulse.spring;

            auto spring_deflection = offset_proj - dof.rest_offset;
            auto spring_force = dof.spring_stiffness * spring_deflection;
            auto spring_impulse = spring_force * dt;
            row.lower_limit = std::min(spring_impulse, scalar(0));
            row.upper_limit = std::max(scalar(0), spring_impulse);

            auto &options = cache.get_options();
            options.error = -spring_deflection / dt;
        }

        // Linear damping and friction.
        if (dof.friction_force > 0 || dof.damping > 0) {
            auto &row = cache.add_row();
            row.J = J;
            row.impulse = dof.applied_impulse.friction_damping;

            auto friction_impulse = dof.friction_force * dt;

            if (dof.damping > 0) {
                auto relspd = get_relative_speed(J, bodyA.linvel, bodyA.angvel, bodyB.linvel, bodyB.angvel);
                friction_impulse += std::abs(relspd) * dof.damping * dt;
            }

            row.lower_limit = -friction_impulse;
            row.upper_limit = friction_impulse;
        }
    }

    auto axisA_x = rotate(bodyA.orn, frame[0].column(0));
    auto axisB_x = rotate(bodyB.orn, frame[1].column(0));

    // Angular.
    for (int i = 0; i < 3; ++i) {
        auto &dof = angular_dofs[i];
        auto non_zero_limit = dof.angle_min < dof.angle_max;
        vector3 axisA, axisB;

        if (i == 0) {
            // Quaternion which rotates the axis of B so it's parallel to the
            // the axis of A.
            auto arc_quat = shortest_arc(axisB_x, axisA_x);

            // Transform a non-axial vector in the frame of B onto A's space so
            // the angular error can be calculated.
            auto angle_axisB = edyn::rotate(conjugate(bodyA.orn) * arc_quat * bodyB.orn, frame[1].column(1));
            dof.current_angle = std::atan2(dot(angle_axisB, frame[0].column(2)),
                                           dot(angle_axisB, frame[0].column(1)));
            axisA = axisA_x;
            axisB = axisB_x;
        } else {
            auto axisA_other = rotate(bodyA.orn, frame[0].column(i == 1 ? 2 : 1));
            auto cos_angle = std::clamp(dot(axisB_x, axisA_other), scalar(-1), scalar(1));
            dof.current_angle = half_pi - std::acos(cos_angle);
            auto axis = cross(axisA_other, axisB_x);

            if (!try_normalize(axis)) {
                axis = i == 1 ? vector3_z : vector3_y;
            }

            axisA = axisB = -axis;
        }

        auto J = std::array<vector3, 4>{vector3_zero, axisA, vector3_zero, -axisB};

        if (dof.limit_enabled) {
            EDYN_ASSERT(!(dof.angle_min > dof.angle_max));

            auto &row = cache.add_row();
            row.J = J;
            row.impulse = dof.applied_impulse.limit;
            auto &options = cache.get_options();

            if (non_zero_limit) {
                auto limit_error = scalar{};
                auto mid_angle = (dof.angle_min + dof.angle_max) / scalar(2);

                if (dof.current_angle < mid_angle) {
                    limit_error = dof.angle_min - dof.current_angle;
                    row.lower_limit = -large_scalar;
                    row.upper_limit = 0;
                } else {
                    limit_error = dof.angle_max - dof.current_angle;
                    row.lower_limit = 0;
                    row.upper_limit = large_scalar;
                }

                options.error = limit_error / dt;
                options.restitution = dof.limit_restitution;
            } else {
                options.error = -dof.current_angle / dt;
                row.lower_limit = -large_scalar;
                row.upper_limit = large_scalar;
            }
        }

        // Angular bump stops.
        if (dof.limit_enabled && non_zero_limit && dof.bump_stop_stiffness > 0 && dof.bump_stop_angle > 0) {
            auto bump_stop_deflection = scalar{0};
            auto bump_stop_min = dof.angle_min + dof.bump_stop_angle;
            auto bump_stop_max = dof.angle_max - dof.bump_stop_angle;

            if (dof.current_angle < bump_stop_min) {
                bump_stop_deflection = dof.current_angle - bump_stop_min;
            } else if (dof.current_angle > bump_stop_max) {
                bump_stop_deflection = dof.current_angle - bump_stop_max;
            }

            auto &row = cache.add_row();
            row.J = J;
            row.impulse = dof.applied_impulse.bump_stop;

            auto spring_force = dof.bump_stop_stiffness * bump_stop_deflection;
            auto spring_impulse = spring_force * dt;
            row.lower_limit = std::min(spring_impulse, scalar(0));
            row.upper_limit = std::max(scalar(0), spring_impulse);

            auto &options = cache.get_options();
            options.error = -bump_stop_deflection / dt;
        }

        // Angular spring.
        if (dof.spring_stiffness > 0) {
            auto &row = cache.add_row();
            row.J = J;
            row.impulse = dof.applied_impulse.spring;

            auto deflection = dof.current_angle - dof.rest_angle;
            auto spring_torque = dof.spring_stiffness * deflection;
            auto spring_impulse = spring_torque * dt;
            row.lower_limit = std::min(spring_impulse, scalar(0));
            row.upper_limit = std::max(scalar(0), spring_impulse);

            auto &options = cache.get_options();
            options.error = -deflection / dt;
        }

        if (dof.friction_torque > 0 || dof.damping > 0) {
            auto &row = cache.add_row();
            row.J = J;
            row.impulse = dof.applied_impulse.friction_damping;

            auto friction_impulse = dof.friction_torque * dt;

            if (dof.damping > 0) {
                auto relvel = dot(bodyA.angvel, axisA) - dot(bodyB.angvel, axisB);
                friction_impulse += std::abs(relvel) * dof.damping * dt;
            }

            row.lower_limit = -friction_impulse;
            row.upper_limit = friction_impulse;
        }
    }
}

void generic_constraint::solve_position(position_solver &solver) {
    for (int i = 0; i < 3; ++i) {
        auto &dof = linear_dofs[i];

        if (!dof.limit_enabled) {
            continue;
        }

        auto originA = solver.get_originA(), originB = solver.get_originB();
        auto &posA = *solver.posA, &posB = *solver.posB;
        auto &ornA = *solver.ornA, &ornB = *solver.ornB;

        auto pivotA = to_world_space(pivot[0], originA, ornA);
        auto pivotB = to_world_space(pivot[1], originB, ornB);
        auto pivot_offset = pivotB - pivotA;
        auto rA = pivotA - posA;
        auto rB = pivotB - posB;

        auto axisA = rotate(ornA, frame[0].column(i));
        auto proj = dot(pivot_offset, axisA);
        auto error = scalar{};

        if (proj < dof.offset_min) {
            error = proj - dof.offset_min;
        } else if (proj > dof.offset_max) {
            error = proj - dof.offset_max;
        }

        solver.solve({axisA, cross(rA, axisA), -axisA, -cross(rB, axisA)}, error);
    }
}

void generic_constraint::store_applied_impulses(const std::vector<scalar> &impulses) {
    unsigned row_idx = 0;

    for (int i = 0; i < 3; ++i) {
        auto &dof = linear_dofs[i];
        auto non_zero_limit = dof.offset_min < dof.offset_max;

        if (dof.limit_enabled) {
            dof.applied_impulse.limit = impulses[row_idx++];
        }

        if (dof.limit_enabled && non_zero_limit && dof.bump_stop_stiffness > 0 && dof.bump_stop_length > 0) {
            dof.applied_impulse.bump_stop = impulses[row_idx++];
        }

        if (dof.spring_stiffness > 0) {
            dof.applied_impulse.spring = impulses[row_idx++];
        }

        if (dof.friction_force > 0 || dof.damping > 0) {
            dof.applied_impulse.friction_damping = impulses[row_idx++];
        }
    }

    for (int i = 0; i < 3; ++i) {
        auto &dof = angular_dofs[i];
        auto non_zero_limit = dof.angle_min < dof.angle_max;

        if (dof.limit_enabled) {
            dof.applied_impulse.limit = impulses[row_idx++];
        }

        if (dof.limit_enabled && non_zero_limit && dof.bump_stop_stiffness > 0 && dof.bump_stop_angle > 0) {
            dof.applied_impulse.bump_stop = impulses[row_idx++];
        }

        if (dof.spring_stiffness > 0) {
            dof.applied_impulse.spring = impulses[row_idx++];
        }

        if (dof.friction_torque > 0 || dof.damping > 0) {
            dof.applied_impulse.friction_damping = impulses[row_idx++];
        }
    }
}

}
