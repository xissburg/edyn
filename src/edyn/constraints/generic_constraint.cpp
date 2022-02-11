#include "edyn/constraints/generic_constraint.hpp"
#include "edyn/constraints/constraint_row.hpp"
#include "edyn/math/constants.hpp"
#include "edyn/math/math.hpp"
#include "edyn/math/matrix3x3.hpp"
#include "edyn/math/transform.hpp"
#include "edyn/comp/position.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/comp/mass.hpp"
#include "edyn/comp/inertia.hpp"
#include "edyn/comp/linvel.hpp"
#include "edyn/comp/angvel.hpp"
#include "edyn/comp/delta_linvel.hpp"
#include "edyn/comp/delta_angvel.hpp"
#include "edyn/comp/origin.hpp"
#include "edyn/comp/tag.hpp"
#include "edyn/dynamics/row_cache.hpp"
#include "edyn/util/constraint_util.hpp"
#include <entt/entity/registry.hpp>

namespace edyn {

void generic_constraint::reset_angles(const quaternion &ornA, const quaternion &ornB) {
    for (size_t i = 0; i < 3; ++i) {
        reset_angle(i, ornA, ornB);
    }
}

void generic_constraint::reset_angle(size_t index, const quaternion &ornA, const quaternion &ornB) {
    angular_dofs[index].current_angle = relative_angle(index, ornA, ornB);
}

scalar generic_constraint::relative_angle(size_t index, const quaternion &ornA, const quaternion &ornB) const {
    auto axisA = rotate(ornA, frame[0].column(index));
    auto axisB = rotate(ornB, frame[1].column(index));
    return relative_angle(index, ornA, ornB, axisA, axisB);
}

scalar generic_constraint::relative_angle(size_t index,
                                          const quaternion &ornA, const quaternion &ornB,
                                          const vector3 &axisA, const vector3 &axisB) const {
    // Quaternion which rotates the axis of B so it's parallel to the
    // the axis of A.
    auto arc_quat = shortest_arc(axisB, axisA);

    // Transform a non-axial vector in the frame of B onto A's space so
    // the angular error can be calculated.
    auto index1 = (index + 1) % 3;
    auto index2 = (index + 2) % 3;
    auto angle_axisB = edyn::rotate(conjugate(ornA) * arc_quat * ornB, frame[1].column(index1));
    return std::atan2(dot(angle_axisB, frame[0].column(index2)),
                      dot(angle_axisB, frame[0].column(index1)));
}

void generic_constraint::update_angle(size_t index, scalar new_angle) {
    auto &dof = angular_dofs[index];
    auto previous_angle = normalize_angle(dof.current_angle);
    // Find shortest path from previous angle to current in
    // the [-π, π] range.
    auto angle_delta0 = new_angle - previous_angle;
    auto angle_delta1 = angle_delta0 + pi2 * to_sign(angle_delta0 < 0);
    auto angle_delta = std::abs(angle_delta0) < std::abs(angle_delta1) ? angle_delta0 : angle_delta1;
    dof.current_angle += angle_delta;
}

template<>
void prepare_constraints<generic_constraint>(entt::registry &registry, row_cache &cache, scalar dt) {
    auto body_view = registry.view<position, orientation,
                                   linvel, angvel,
                                   mass_inv, inertia_world_inv,
                                   delta_linvel, delta_angvel>();
    auto con_view = registry.view<generic_constraint>(entt::exclude_t<disabled_tag>{});
    auto origin_view = registry.view<origin>();

    con_view.each([&] (generic_constraint &con) {
        auto [posA, ornA, linvelA, angvelA, inv_mA, inv_IA, dvA, dwA] = body_view.get(con.body[0]);
        auto [posB, ornB, linvelB, angvelB, inv_mB, inv_IB, dvB, dwB] = body_view.get(con.body[1]);

        auto originA = origin_view.contains(con.body[0]) ? origin_view.get<origin>(con.body[0]) : static_cast<vector3>(posA);
        auto originB = origin_view.contains(con.body[1]) ? origin_view.get<origin>(con.body[1]) : static_cast<vector3>(posB);

        auto pivotA = to_world_space(con.pivot[0], originA, ornA);
        auto pivotB = to_world_space(con.pivot[1], originB, ornB);
        auto rA = pivotA - posA;
        auto rB = pivotB - posB;

        auto pivot_offset = pivotB - pivotA;
        auto row_idx = size_t{};

        // Linear.
        for (int i = 0; i < 3; ++i) {
            const auto &dof = con.linear_dofs[i];

            if (!dof.enabled) {
                continue;
            }

            auto axisA = rotate(ornA, con.frame[0].column(i));
            auto J = std::array<vector3, 4>{axisA, cross(rA, axisA), -axisA, -cross(rB, axisA)};

            auto &row = cache.rows.emplace_back();
            row.J = J;
            row.inv_mA = inv_mA; row.inv_IA = inv_IA;
            row.inv_mB = inv_mB; row.inv_IB = inv_IB;
            row.dvA = &dvA; row.dwA = &dwA;
            row.dvB = &dvB; row.dwB = &dwB;
            row.impulse = con.impulse[row_idx++];

            auto options = constraint_row_options{};

            auto has_limit = dof.offset_min < dof.offset_max;
            auto offset_proj = dot(pivot_offset, axisA);

            if (has_limit) {
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

            prepare_row(row, options, linvelA, angvelA, linvelB, angvelB);
            warm_start(row);

            // Linear bump stops.
            if (has_limit && dof.bump_stop_stiffness > 0 && dof.bump_stop_length > 0) {
                auto bump_stop_deflection = scalar{};
                auto bump_stop_min = dof.offset_min + dof.bump_stop_length;
                auto bump_stop_max = dof.offset_max - dof.bump_stop_length;

                if (offset_proj < bump_stop_min) {
                    bump_stop_deflection = offset_proj - bump_stop_min;
                } else if (offset_proj > bump_stop_max) {
                    bump_stop_deflection = offset_proj - bump_stop_max;
                }

                auto &row = cache.rows.emplace_back();
                row.J = J;
                row.inv_mA = inv_mA; row.inv_IA = inv_IA;
                row.inv_mB = inv_mB; row.inv_IB = inv_IB;
                row.dvA = &dvA; row.dwA = &dwA;
                row.dvB = &dvB; row.dwB = &dwB;
                row.impulse = con.impulse[row_idx++];

                auto spring_force = dof.bump_stop_stiffness * bump_stop_deflection;
                auto spring_impulse = spring_force * dt;
                row.lower_limit = std::min(spring_impulse, scalar(0));
                row.upper_limit = std::max(scalar(0), spring_impulse);

                auto options = constraint_row_options{};
                options.error = -bump_stop_deflection / dt;

                prepare_row(row, options, linvelA, angvelA, linvelB, angvelB);
                warm_start(row);
            }

            // Linear spring.
            if (dof.spring_stiffness > 0) {
                auto &row = cache.rows.emplace_back();
                row.J = J;
                row.inv_mA = inv_mA; row.inv_IA = inv_IA;
                row.inv_mB = inv_mB; row.inv_IB = inv_IB;
                row.dvA = &dvA; row.dwA = &dwA;
                row.dvB = &dvB; row.dwB = &dwB;
                row.impulse = con.impulse[row_idx++];

                auto spring_deflection = offset_proj - dof.rest_offset;
                auto spring_force = dof.spring_stiffness * spring_deflection;
                auto spring_impulse = spring_force * dt;
                row.lower_limit = std::min(spring_impulse, scalar(0));
                row.upper_limit = std::max(scalar(0), spring_impulse);

                auto options = constraint_row_options{};
                options.error = -spring_deflection / dt;

                prepare_row(row, options, linvelA, angvelA, linvelB, angvelB);
                warm_start(row);
            }

            // Linear damping and friction.
            if (dof.friction_force > 0 || dof.damping > 0) {
                auto &row = cache.rows.emplace_back();
                row.J = J;
                row.inv_mA = inv_mA; row.inv_IA = inv_IA;
                row.inv_mB = inv_mB; row.inv_IB = inv_IB;
                row.dvA = &dvA; row.dwA = &dwA;
                row.dvB = &dvB; row.dwB = &dwB;
                row.impulse = con.impulse[row_idx++];

                auto friction_impulse = dof.friction_force * dt;

                if (dof.damping > 0) {
                    auto relspd = get_relative_speed(J, linvelA, angvelA, linvelB, angvelB);
                    friction_impulse += std::abs(relspd) * dof.damping * dt;
                }

                row.lower_limit = -friction_impulse;
                row.upper_limit = friction_impulse;

                prepare_row(row, {}, linvelA, angvelA, linvelB, angvelB);
                warm_start(row);
            }
        }

        // Angular.
        for (int i = 0; i < 3; ++i) {
            const auto &dof = con.angular_dofs[i];

            if (!dof.enabled) {
                continue;
            }

            auto axisA = rotate(ornA, con.frame[0].column(i));
            auto axisB = rotate(ornB, con.frame[1].column(i));
            auto J = std::array<vector3, 4>{vector3_zero, axisA, vector3_zero, -axisB};

            auto &row = cache.rows.emplace_back();
            row.J = J;
            row.inv_mA = inv_mA; row.inv_IA = inv_IA;
            row.inv_mB = inv_mB; row.inv_IB = inv_IB;
            row.dvA = &dvA; row.dwA = &dwA;
            row.dvB = &dvB; row.dwB = &dwB;
            row.impulse = con.impulse[row_idx++];

            auto angle = con.relative_angle(i, ornA, ornB, axisA, axisB);
            con.update_angle(i, angle);

            auto has_limit = dof.angle_min < dof.angle_max;
            auto options = constraint_row_options{};

            if (has_limit) {

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
                auto error = -dof.current_angle;
                options.error = error / dt;
                row.lower_limit = -large_scalar;
                row.upper_limit = large_scalar;
            }

            prepare_row(row, options, linvelA, angvelA, linvelB, angvelB);
            warm_start(row);

            // Angular bump stops.
            if (has_limit && dof.bump_stop_stiffness > 0 && dof.bump_stop_angle > 0) {
                auto bump_stop_deflection = scalar{0};
                auto bump_stop_min = dof.angle_min + dof.bump_stop_angle;
                auto bump_stop_max = dof.angle_max - dof.bump_stop_angle;

                if (dof.current_angle < bump_stop_min) {
                    bump_stop_deflection = dof.current_angle - bump_stop_min;
                } else if (dof.current_angle > bump_stop_max) {
                    bump_stop_deflection = dof.current_angle - bump_stop_max;
                }

                auto &row = cache.rows.emplace_back();
                row.J = J;
                row.inv_mA = inv_mA; row.inv_IA = inv_IA;
                row.inv_mB = inv_mB; row.inv_IB = inv_IB;
                row.dvA = &dvA; row.dwA = &dwA;
                row.dvB = &dvB; row.dwB = &dwB;
                row.impulse = con.impulse[row_idx++];

                auto spring_force = dof.bump_stop_stiffness * bump_stop_deflection;
                auto spring_impulse = spring_force * dt;
                row.lower_limit = std::min(spring_impulse, scalar(0));
                row.upper_limit = std::max(scalar(0), spring_impulse);

                auto options = constraint_row_options{};
                options.error = -bump_stop_deflection / dt;

                prepare_row(row, options, linvelA, angvelA, linvelB, angvelB);
                warm_start(row);
            }

            // Angular spring.
            if (has_limit && dof.spring_stiffness > 0) {
                auto &row = cache.rows.emplace_back();
                row.J = J;
                row.inv_mA = inv_mA; row.inv_IA = inv_IA;
                row.inv_mB = inv_mB; row.inv_IB = inv_IB;
                row.dvA = &dvA; row.dwA = &dwA;
                row.dvB = &dvB; row.dwB = &dwB;
                row.impulse = con.impulse[row_idx++];

                auto deflection = dof.current_angle - dof.rest_angle;
                auto spring_torque = dof.spring_stiffness * deflection;
                auto spring_impulse = spring_torque * dt;
                row.lower_limit = std::min(spring_impulse, scalar(0));
                row.upper_limit = std::max(scalar(0), spring_impulse);

                auto options = constraint_row_options{};
                options.error = -deflection / dt;

                prepare_row(row, options, linvelA, angvelA, linvelB, angvelB);
                warm_start(row);
            }

            if (has_limit && (dof.friction_torque > 0 || dof.damping > 0)) {
                auto &row = cache.rows.emplace_back();
                row.J = J;
                row.inv_mA = inv_mA; row.inv_IA = inv_IA;
                row.inv_mB = inv_mB; row.inv_IB = inv_IB;
                row.dvA = &dvA; row.dwA = &dwA;
                row.dvB = &dvB; row.dwB = &dwB;
                row.impulse = con.impulse[row_idx++];

                auto friction_impulse = dof.friction_torque * dt;

                if (dof.damping > 0) {
                    auto relvel = dot(angvelA, axisA) - dot(angvelB, axisB);
                    friction_impulse += std::abs(relvel) * dof.damping * dt;
                }

                row.lower_limit = -friction_impulse;
                row.upper_limit = friction_impulse;

                prepare_row(row, {}, linvelA, angvelA, linvelB, angvelB);
                warm_start(row);
            }
        }

        cache.con_num_rows.push_back(row_idx);
    });
}

template<>
bool solve_position_constraints<generic_constraint>(entt::registry &registry, scalar dt) {
    auto con_view = registry.view<generic_constraint>(entt::exclude_t<disabled_tag>{});
    auto body_view = registry.view<position, orientation, mass_inv, inertia_world_inv>();
    auto origin_view = registry.view<origin>();
    auto linear_error = scalar(0);
    auto angular_error = scalar(0);

    con_view.each([&] (generic_constraint &con) {
        auto [posA, ornA, inv_mA, inv_IA] = body_view.get(con.body[0]);
        auto [posB, ornB, inv_mB, inv_IB] = body_view.get(con.body[1]);

        auto originA = origin_view.contains(con.body[0]) ? origin_view.get<origin>(con.body[0]) : static_cast<vector3>(posA);
        auto originB = origin_view.contains(con.body[1]) ? origin_view.get<origin>(con.body[1]) : static_cast<vector3>(posB);

        auto pivotA = to_world_space(con.pivot[0], originA, ornA);
        auto pivotB = to_world_space(con.pivot[1], originB, ornB);
        auto pivot_offset = pivotB - pivotA;

        for (int i = 0; i < 3; ++i) {
            const auto &dof = con.linear_dofs[i];

            if (!dof.enabled) {
                continue;
            }

            auto axisA = rotate(ornA, con.frame[0].column(i));
            auto proj = dot(pivot_offset, axisA);
            auto error = scalar{};

            if (proj < dof.offset_min) {
                error = proj - dof.offset_min;
            } else if (proj > dof.offset_max) {
                error = proj - dof.offset_max;
            }

            auto rA = pivotA - posA;
            auto rB = pivotB - posB;
            auto J = std::array<vector3, 4>{axisA, cross(rA, axisA), -axisA, -cross(rB, axisA)};
            auto eff_mass = get_effective_mass(J, inv_mA, inv_IA, inv_mB, inv_IB);
            auto correction = error * eff_mass;

            posA += inv_mA * J[0] * correction;
            posB += inv_mB * J[2] * correction;
            ornA += quaternion_derivative(ornA, inv_IA * J[1] * correction);
            ornB += quaternion_derivative(ornB, inv_IB * J[3] * correction);
            ornA = normalize(ornA);
            ornB = normalize(ornB);

            linear_error = std::max(std::abs(error), linear_error);

            auto basisA = to_matrix3x3(ornA);
            inv_IA = basisA * inv_IA * transpose(basisA);

            auto basisB = to_matrix3x3(ornB);
            inv_IB = basisB * inv_IB * transpose(basisB);
        }
    });

    if (linear_error < scalar(0.005) && angular_error < to_radians(2)) {
        return true;
    }

    return false;
}

}
