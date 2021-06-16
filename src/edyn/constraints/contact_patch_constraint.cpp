#include "edyn/constraints/contact_patch_constraint.hpp"
#include "edyn/constraints/constraint_row.hpp"
#include "edyn/constraints/constraint_impulse.hpp"
#include "edyn/dynamics/row_cache.hpp"
#include "edyn/collision/contact_manifold.hpp"
#include "edyn/collision/contact_point.hpp"
#include "edyn/comp/position.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/comp/mass.hpp"
#include "edyn/comp/inertia.hpp"
#include "edyn/comp/linvel.hpp"
#include "edyn/comp/angvel.hpp"
#include "edyn/comp/delta_linvel.hpp"
#include "edyn/comp/delta_angvel.hpp"
#include "edyn/comp/spin.hpp"
#include "edyn/util/tire_util.hpp"
#include "edyn/util/constraint_util.hpp"
#include "edyn/math/matrix3x3.hpp"
#include "edyn/math/math.hpp"
#include "edyn/shapes/cylinder_shape.hpp"
#include <entt/entt.hpp>

namespace edyn {

struct row_start_index_contact_patch_constraint {
    size_t value;
};

void contact_patch_constraint::clear() {
    for (size_t i = 0; i < num_tread_rows; ++i) {
        auto &tread_row = m_tread_rows[i];
        tread_row.bristles.clear();
    }
}

template<>
void prepare_constraints<contact_patch_constraint>(entt::registry &registry, row_cache &cache, scalar dt) {
    auto body_view = registry.view<position, orientation,
                                   linvel, angvel,
                                   mass_inv, inertia_world_inv,
                                   delta_linvel, delta_angvel>();
    auto con_view = registry.view<contact_patch_constraint>();
    auto cp_view = registry.view<contact_point>();
    auto imp_view = registry.view<constraint_impulse>();

    size_t start_idx = cache.rows.size();
    registry.ctx_or_set<row_start_index_contact_patch_constraint>().value = start_idx;

    con_view.each([&] (entt::entity entity, contact_patch_constraint &con) {
        auto &cp = cp_view.get(entity);

        if (cp.distance > 0) {
            con.clear();
            return;
        }

        auto [posA, ornA, linvelA, angvelA, inv_mA, inv_IA, dvA, dwA] =
            body_view.get<position, orientation, linvel, angvel, mass_inv, inertia_world_inv, delta_linvel, delta_angvel>(con.body[0]);
        auto [posB, ornB, linvelB, angvelB, inv_mB, inv_IB, dvB, dwB] =
            body_view.get<position, orientation, linvel, angvel, mass_inv, inertia_world_inv, delta_linvel, delta_angvel>(con.body[1]);
        auto &imp = imp_view.get(entity);

        // Wheel spin axis in world space.
        const auto axis = quaternion_x(ornA);

        const auto &spin_angleA = registry.get<spin_angle>(con.body[0]);
        const auto spin_ornA = ornA * quaternion_axis_angle(vector3_x, spin_angleA);
        const auto &spinA = registry.get<spin>(con.body[0]);
        const auto spin_angvelA = angvelA + axis * spinA;

        const auto ornB_conj = conjugate(ornB);

        const auto &cyl = registry.get<cylinder_shape>(con.body[0]);

        const auto normal = rotate(ornB, cp.normalB);

        // Calculate contact patch width.
        con.m_sin_camber = dot(axis, normal);
        auto camber_angle = std::asin(con.m_sin_camber);
        auto normalized_contact_width = std::cos(std::atan(std::pow(std::abs(camber_angle), std::log(-cp.distance * 300 + 1))));
        con.m_contact_width = cyl.half_length * 2 * normalized_contact_width;

        // Calculate center of pressure.
        const auto axis_hl = axis * cyl.half_length;
        auto normalized_center_offset = -std::sin(std::atan(camber_angle));

        // Where the row starts in the x-axis in object space.
        auto row_start = con.m_sin_camber < 0 ? -cyl.half_length : cyl.half_length - con.m_contact_width;

        // A point on the contact plane.
        auto pivotB = posB + rotate(ornB, cp.pivotB);

        // Intersect lines going from the circle center to the support point with the
        // contact plane to find the initial contact extent.
        auto circle_center0 = posA - axis_hl;
        auto circle_center1 = posA + axis_hl;
        auto sup0 = support_point_circle(posA - axis_hl, ornA, cyl.radius, -normal);
        auto sup1 = sup0 + axis_hl * 2; // because circles are parallel
        auto intersection0 = intersect_line_plane(circle_center0, sup0 - circle_center0, pivotB, normal);
        auto intersection1 = intersect_line_plane(circle_center1, sup1 - circle_center1, pivotB, normal);
        auto cyl_len_inv = scalar(1) / (scalar(2) * cyl.half_length);
        auto plane_point0 = lerp(intersection0, intersection1, (row_start + cyl.half_length) * cyl_len_inv);
        auto plane_point1 = lerp(intersection0, intersection1, (row_start + cyl.half_length + con.m_contact_width) * cyl_len_inv);
        auto center_lerp_param = (normalized_center_offset + scalar(1)) * scalar(0.5);
        auto contact_center = lerp(plane_point0, plane_point1, center_lerp_param);
        auto geometric_center = lerp(plane_point0, plane_point1, scalar(0.5));
        const auto tire_y = quaternion_y(ornA);
        const auto tire_up = dot(tire_y, normal) > 0 ? tire_y : -tire_y;
        auto deflection0 = std::max(dot(intersection0 - sup0, tire_up), scalar(0));
        auto deflection1 = std::max(dot(intersection1 - sup1, tire_up), scalar(0));

        con.m_center = geometric_center;
        con.m_pivot = contact_center;
        con.m_normal = normal;
        con.m_deflection = -cp.distance;

        // Setup non-penetration constraint.
        auto rA = contact_center - posA;
        auto rB = contact_center - posB;

        auto first_row_idx = cache.rows.size();

        // Normal spring.
        {
            auto linvelrel = linvelA - linvelB;
            auto speed = length(linvelrel - normal * dot(linvelrel, normal));
            auto stiffness = velocity_dependent_vertical_stiffness(con.m_normal_stiffness, speed);

            auto normal_spring_force = con.m_deflection * stiffness;
            auto normal_spring_impulse = normal_spring_force * dt;

            auto &row = cache.rows.emplace_back();
            row.J = {normal, cross(rA, normal), -normal, -cross(rB, normal)};
            row.lower_limit = 0;
            row.upper_limit = std::max(scalar(0), normal_spring_impulse);

            auto options = constraint_row_options{};
            options.error = normal_spring_impulse > 0 ? -large_scalar : large_scalar;

            row.inv_mA = inv_mA; row.inv_IA = inv_IA;
            row.inv_mB = inv_mB; row.inv_IB = inv_IB;
            row.dvA = &dvA; row.dwA = &dwA;
            row.dvB = &dvB; row.dwB = &dwB;
            row.impulse = imp.values[0];

            prepare_row(row, options, linvelA, linvelB, angvelA, angvelB);
            warm_start(row);
        }

        // Normal damping.
        {
            auto &row = cache.rows.emplace_back();
            row.J = {normal, cross(rA, normal), -normal, -cross(rB, normal)};

            auto normal_relspd = dot(row.J[0], linvelA) +
                                 dot(row.J[1], angvelA) +
                                 dot(row.J[2], linvelB) +
                                 dot(row.J[3], angvelB);
            auto normal_damper_force = con.m_normal_damping * normal_relspd;
            auto normal_damper_impulse = std::abs(normal_damper_force * dt);
            con.m_normal_relspd = normal_relspd;

            row.lower_limit = 0;
            row.upper_limit = normal_damper_impulse;

            row.inv_mA = inv_mA; row.inv_IA = inv_IA;
            row.inv_mB = inv_mB; row.inv_IB = inv_IB;
            row.dvA = &dvA; row.dwA = &dwA;
            row.dvB = &dvB; row.dwB = &dwB;
            row.impulse = imp.values[1];

            prepare_row(row, {}, linvelA, linvelB, angvelA, angvelB);
            warm_start(row);
        }

        auto forward = cross(axis, normal);
        auto forward_len_sqr = length_sqr(forward);

        if (forward_len_sqr > EDYN_EPSILON) {
            forward /= std::sqrt(forward_len_sqr);
        } else {
            forward = quaternion_z(ornA);
        }

        // Calculate longitudinal and lateral friction directions.
        con.m_lat_dir = axis - normal * dot(axis, normal);
        auto lat_dir_len2 = length_sqr(con.m_lat_dir);

        if (lat_dir_len2 > EDYN_EPSILON) {
            con.m_lat_dir /= std::sqrt(lat_dir_len2);
            con.m_lon_dir = cross(con.m_lat_dir, normal);
        } else {
            auto tire_z = quaternion_z(ornA);
            con.m_lon_dir = tire_z - normal * dot(tire_z, normal);
            con.m_lon_dir = normalize(con.m_lon_dir);
            con.m_lat_dir = cross(normal, con.m_lon_dir);
        }

        auto tread_width = con.m_contact_width / con.num_tread_rows;
        auto r0_inv = scalar(1) / cyl.radius;

        const scalar half_perimeter = pi * cyl.radius;
        const scalar perimeter = half_perimeter * 2;
        const scalar desired_spacing = 0.01;
        const uint16_t num_bristles = std::floor(perimeter / desired_spacing);
        const scalar bristle_spacing = perimeter / num_bristles;
        const scalar bristle_angle_delta = bristle_spacing * r0_inv;

        // Support point angle in circle space where z points forward and y is up.
        // It allows us to observe the contact point location with respect to the
        // current spin angle and determine which bristles lie within the contact
        // patch range.
        auto sup0_obj = to_object_space(sup0, posA, ornA);
        scalar contact_angle = std::atan2(sup0_obj.y, sup0_obj.z);

        // Transform angle from [-π, π] to [0, 2π].
        if (contact_angle < 0) {
            contact_angle += 2 * pi;
        }

        contact_angle += spin_angleA.s;

        auto &normal_spring_row = cache.rows[first_row_idx];
        auto &normal_damping_row = cache.rows[first_row_idx + 1];
        auto normal_force = (normal_spring_row.impulse + normal_damping_row.impulse) / dt;

        // Accumulate forces and errors along all bristles.
        auto lon_force = scalar {0};
        auto lat_force = scalar {0};
        auto aligning_torque = scalar {0};
        auto lon_damping = scalar {0};
        auto lat_damping = scalar {0};
        auto aligning_damping = scalar {0};
        con.m_contact_len_avg = 0;

        // Update bristles for each row.
        for (size_t i = 0; i < con.num_tread_rows; ++i) {
            auto &tread_row = con.m_tread_rows[i];
            auto row_x = row_start + tread_width * scalar(i + 0.5);

            // Normal deflection and length for this row of bristles.
            const auto row_proportion = (row_x - row_start) / con.m_contact_width;
            const auto defl = lerp(deflection0, deflection1, row_proportion);
            const auto row_half_length = std::min(scalar(0.4) * cyl.radius *
                                                  (defl * r0_inv + scalar(2.25) *
                                                  std::sqrt(defl * r0_inv)),
                                                  cyl.radius * scalar(0.9));
            const auto row_half_angle = std::asin(row_half_length / cyl.radius);

            con.m_contact_len_avg += row_half_length * 2;

            tread_row.tread_width = tread_width;
            tread_row.patch_half_length = row_half_length;

            // Contact patch extents in radians for this row.
            auto patch_start_angle = contact_angle - row_half_angle;
            auto patch_end_angle   = contact_angle + row_half_angle;

            // Angle of the first bristle without wrapping around in the [0, 2π] range.
            const scalar start_angle = std::ceil(patch_start_angle / bristle_angle_delta) * bristle_angle_delta;

            // Index of the first and last bristles.
            const uint16_t start_idx = uint16_t(std::ceil(patch_start_angle / bristle_angle_delta)) % num_bristles;
            const uint16_t end_idx = uint16_t(std::ceil(patch_end_angle / bristle_angle_delta)) % num_bristles;
            const auto wraps_around = start_idx > end_idx;
            uint16_t row_num_bristles = wraps_around ? num_bristles - start_idx + end_idx : end_idx - start_idx;

            // Remove existing bristles that are outside the contact patch.
            for (auto iter = tread_row.bristles.begin(); iter != tread_row.bristles.end();) {
                auto idx = iter->first;
                if ((!wraps_around && (idx < start_idx || idx >= end_idx)) ||
                    ( wraps_around && (idx < start_idx && idx >= end_idx))) {
                    iter = tread_row.bristles.erase(iter);
                } else {
                    ++iter;
                }
            }

            const auto row_area = scalar(2) * row_half_length * tread_width;
            const auto tread_area = row_num_bristles > 0 ? row_area / row_num_bristles : scalar(0);
            const auto spin_count_delta = spin_angleA.count - tread_row.prev_spin_count;

            // Previous contact patch range accounting for the number of spins
            // since the last update so that the ranges and angles are all laid
            // out in a segment without wrapping around.
            auto prev_contact_angle     = tread_row.prev_contact_angle - spin_count_delta * pi2;
            auto prev_patch_start_angle = prev_contact_angle - tread_row.prev_row_half_angle;
            auto prev_patch_end_angle   = prev_contact_angle + tread_row.prev_row_half_angle;

            auto prev_bristle_defl = vector3_zero;

            // Update persisted bristles and introduce new bristles into the
            // contact patch.
            for (uint16_t j = 0; j < row_num_bristles; ++j) {
                const auto bristle_idx = (start_idx + j) % num_bristles;
                contact_patch_constraint::brush_bristle *bristle;

                if (tread_row.bristles.count(bristle_idx)) {
                    bristle = &tread_row.bristles[bristle_idx];
                } else {
                    auto bristle_angle = start_angle + scalar(j) * bristle_angle_delta;
                    // Bristle coordinates in object space.
                    auto bristle_pivot = vector3{row_x,
                                                std::sin(bristle_angle) * cyl.radius,
                                                std::cos(bristle_angle) * cyl.radius};

                    // Calculate time when bristle came into contact with ground,
                    // i.e. the moment it entered the contact patch.
                    scalar entry_dt = 0;

                    // Check whether it entered through the front or rear of the patch
                    // and `entry_dt` is going to be a negative proportion of `dt`.
                    if (bristle_angle < prev_patch_start_angle) {
                        auto denom = prev_patch_start_angle - patch_start_angle;
                        if (std::abs(denom) > EDYN_EPSILON) {
                            entry_dt = -dt * (bristle_angle - patch_start_angle) / denom;
                        }
                    } else if (bristle_angle > prev_patch_end_angle) {
                        auto denom = prev_patch_end_angle - patch_end_angle;
                        if (std::abs(denom) > EDYN_EPSILON) {
                            entry_dt = -dt * (bristle_angle - patch_end_angle) / denom;
                        }
                    }

                    // Rollback in time.
                    auto entry_posA = posA + linvelA * entry_dt;
                    auto entry_ornA = integrate(ornA, angvelA, entry_dt);
                    auto entry_angleA = spin_angleA + spinA * entry_dt;
                    auto entry_spin_ornA = entry_ornA * quaternion_axis_angle(vector3_x, entry_angleA);

                    // World-space position where the bristle entered the contact patch.
                    auto entry_bristle_pos = project_plane(entry_posA + rotate(entry_spin_ornA, bristle_pivot), pivotB, normal);

                    // Calculate pivots at time of entry.
                    auto entry_posB = posB + linvelB * entry_dt;
                    auto entry_ornB = integrate(ornB, angvelB, entry_dt);
                    auto rB = rotate(conjugate(entry_ornB), entry_bristle_pos - entry_posB);
                    auto rA = bristle_pivot;

                    tread_row.bristles[bristle_idx] = contact_patch_constraint::brush_bristle {rA, rB};
                    bristle = &tread_row.bristles[bristle_idx];

                    // Set tip position to current.
                    bristle->tip = project_plane(posB + rotate(ornB, rB), pivotB, normal);
                }

                // Bristle root and tip in world space, on the contact plane.
                auto bristle_root = project_plane(posA + rotate(spin_ornA, bristle->pivotA), pivotB, normal);
                auto bristle_tip = project_plane(posB + rotate(ornB, bristle->pivotB), pivotB, normal);

                auto normal_pressure = row_half_length > 0 ?
                                    (normal_force / con.num_tread_rows) /
                                    (tread_width * 2 * row_half_length) :
                                    scalar(0);
                auto mu0 = cp.friction * std::exp(scalar(-0.001) * con.m_load_sensitivity * normal_force);
                bristle->friction = mu0 / (1 + con.m_speed_sensitivity * bristle->sliding_spd);

                auto spring_force = vector3_zero;
                auto bristle_defl = bristle_root - bristle_tip;
                auto bristle_defl_len2 = length_sqr(bristle_defl);

                if (bristle_defl_len2 > EDYN_EPSILON) {
                    // TODO: handle anysotropic stiffness.
                    // Calculate tread length. Generally that would be `bristle_angle_delta * cyl.radius`
                    // but for the first tread the length is gonna be smaller, i.e. from the
                    // `patch_start_angle` until the bristle angle.
                    auto a0 = j > 0 ? start_angle + scalar(j - 1) * bristle_angle_delta : patch_start_angle;
                    auto a1 = start_angle + scalar(j) * bristle_angle_delta;
                    auto x0 = a0 * cyl.radius;
                    auto x1 = a1 * cyl.radius;
                    auto dx = x1 - x0;

                    // The force is calculated as an integral from the previous deflection until the
                    // current along the row.
                    spring_force = con.m_lon_tread_stiffness * tread_width *
                                    (prev_bristle_defl * dx + (bristle_defl - prev_bristle_defl) * dx * scalar(0.5));

                    // The area of the first tread is a proportion of the total.
                    auto local_area = j > 0 ? tread_area :
                                    scalar(0.5) * tread_area * (a1 - a0) / bristle_angle_delta;
                    auto max_friction_force = bristle->friction * normal_pressure * local_area;

                    // Bristle deflection force is greater than maximum friction force
                    // for the current normal load, which mean the bristle must slide.
                    // Thus, move the bristle tip closer to its root so that the
                    // tangential deflection force is equals to the friction force.
                    if (length_sqr(spring_force) > max_friction_force * max_friction_force) {
                        auto error = std::sqrt(bristle_defl_len2);
                        auto dir = bristle_defl / error;
                        auto max_tread_defl = bristle->friction * normal_pressure / con.m_lon_tread_stiffness;
                        bristle_defl = dir * max_tread_defl;
                        spring_force = con.m_lon_tread_stiffness * tread_width *
                                    (prev_bristle_defl * dx + (bristle_defl - prev_bristle_defl) * dx * scalar(0.5));
                        bristle_tip = bristle_root - bristle_defl;

                        // Move pivot in B to match new tip location.
                        bristle->pivotB = rotate(ornB_conj, bristle_tip - posB);
                    }
                }

                // Calculate bristle tip velocity by subtracting current from previous
                // position and dividing by the elapsed time.
                auto tip_vel = project_direction((bristle_tip - bristle->tip) / dt, normal);
                auto sliding_spd = length(tip_vel);
                bristle->sliding_spd = sliding_spd;

                auto root_vel = project_direction(linvelA + cross(spin_angvelA, bristle_root - posA), normal);
                auto bristle_defl_vel = tip_vel - root_vel;

                auto damping_force = bristle_defl_vel * con.m_tread_damping * tread_area;
                lon_damping += dot(con.m_lon_dir, damping_force);
                lat_damping += dot(con.m_lat_dir, damping_force);
                aligning_damping += dot(cross(bristle_root - contact_center, damping_force), normal);

                lon_force += dot(con.m_lon_dir, -spring_force);
                lat_force += dot(con.m_lat_dir, -spring_force);
                aligning_torque += dot(cross(bristle_root - contact_center, -spring_force), normal);

                bristle->deflection = bristle_defl;
                bristle->tip = bristle_tip;
                bristle->root = bristle_root;

                prev_bristle_defl = bristle_defl;

                // Add force from last bristle until end of patch.
                if (j == row_num_bristles - 1) {
                    auto x0 = (start_angle + scalar(j) * bristle_angle_delta) * cyl.radius;
                    auto x1 = patch_end_angle * cyl.radius;
                    auto dx = x1 - x0;

                    spring_force = con.m_lon_tread_stiffness * tread_width *
                                    (prev_bristle_defl * dx + (bristle_defl - prev_bristle_defl) * dx * scalar(0.5));
                    lon_force += dot(con.m_lon_dir, -spring_force);
                    lat_force += dot(con.m_lat_dir, -spring_force);
                    aligning_torque += dot(cross(bristle_root - contact_center, -spring_force), normal);
                }
            }

            tread_row.tread_area = tread_area;
            tread_row.prev_contact_angle = contact_angle;
            tread_row.prev_spin_count = spin_angleA.count;
            tread_row.prev_row_half_angle = row_half_angle;
        }

        con.m_contact_len_avg /= con.num_tread_rows;

        size_t total_bristles = 0;
        con.m_sliding_spd_avg = scalar {0};

        for (auto &row : con.m_tread_rows) {
            for (auto &pair : row.bristles) {
                auto &bristle = pair.second;
                con.m_sliding_spd_avg += bristle.sliding_spd;
                ++total_bristles;
            }
        }

        if (total_bristles > 0) {
            con.m_sliding_spd_avg /= total_bristles;
        }

        // Longitudinal stiffness.
        {
            auto p = cross(rA, con.m_lon_dir);
            auto q = cross(rB, con.m_lon_dir);
            auto spring_impulse = lon_force * dt;

            auto &row = cache.rows.emplace_back();
            row.J = {con.m_lon_dir, p, -con.m_lon_dir, -q};
            row.lower_limit = std::min(spring_impulse, scalar(0));
            row.upper_limit = std::max(scalar(0), spring_impulse);
            row.inv_mA = inv_mA; row.inv_IA = inv_IA;
            row.inv_mB = inv_mB; row.inv_IB = inv_IB;
            row.dvA = &dvA; row.dwA = &dwA;
            row.dvB = &dvB; row.dwB = &dwB;
            row.impulse = imp_view.get(entity).values[0];
            row.use_spin[0] = true;
            row.use_spin[1] = true;

            auto options = constraint_row_options{};
            options.error = spring_impulse > 0 ? -large_scalar : large_scalar;

            prepare_row(row, options, linvelA, linvelB, angvelA, angvelB);
            warm_start(row);
        }

        // Longitudinal damping.
        {
            auto p = cross(rA, con.m_lon_dir);
            auto q = cross(rB, con.m_lon_dir);
            auto damping_impulse = lon_damping * dt;
            auto impulse = std::abs(damping_impulse);

            auto &row = cache.rows.emplace_back();
            row.J = {con.m_lon_dir, p, -con.m_lon_dir, -q};
            row.lower_limit = -impulse;
            row.upper_limit =  impulse;
            row.inv_mA = inv_mA; row.inv_IA = inv_IA;
            row.inv_mB = inv_mB; row.inv_IB = inv_IB;
            row.dvA = &dvA; row.dwA = &dwA;
            row.dvB = &dvB; row.dwB = &dwB;
            row.impulse = imp_view.get(entity).values[0];
            row.use_spin[0] = true;
            row.use_spin[1] = true;

            prepare_row(row, {}, linvelA, linvelB, angvelA, angvelB);
            warm_start(row);

            con.m_lon_damping = lon_damping;
        }

        // Lateral stiffness.
        {
            auto p = cross(rA, con.m_lat_dir);
            auto q = cross(rB, con.m_lat_dir);
            auto spring_impulse = lat_force * dt;

            auto &row = cache.rows.emplace_back();
            row.J = {con.m_lat_dir, p, -con.m_lat_dir, -q};
            row.lower_limit = std::min(spring_impulse, scalar(0));
            row.upper_limit = std::max(scalar(0), spring_impulse);
            row.inv_mA = inv_mA; row.inv_IA = inv_IA;
            row.inv_mB = inv_mB; row.inv_IB = inv_IB;
            row.dvA = &dvA; row.dwA = &dwA;
            row.dvB = &dvB; row.dwB = &dwB;
            row.impulse = imp_view.get(entity).values[0];

            auto options = constraint_row_options{};
            options.error = spring_impulse > 0 ? -large_scalar : large_scalar;

            prepare_row(row, options, linvelA, linvelB, angvelA, angvelB);
            warm_start(row);
        }

        // Lateral damping.
        {
            auto p = cross(rA, con.m_lat_dir);
            auto q = cross(rB, con.m_lat_dir);
            auto damping_impulse = lat_damping * dt;
            auto impulse = std::abs(damping_impulse);

            auto &row = cache.rows.emplace_back();
            row.J = {con.m_lat_dir, p, -con.m_lat_dir, -q};
            row.lower_limit = -impulse;
            row.upper_limit =  impulse;
            row.inv_mA = inv_mA; row.inv_IA = inv_IA;
            row.inv_mB = inv_mB; row.inv_IB = inv_IB;
            row.dvA = &dvA; row.dwA = &dwA;
            row.dvB = &dvB; row.dwB = &dwB;
            row.impulse = imp_view.get(entity).values[0];

            prepare_row(row, {}, linvelA, linvelB, angvelA, angvelB);
            warm_start(row);

            con.m_lat_damping = lat_damping;
        }

        // Aligning moment.
        {
            // TODO: Also apply force on center of mass due to torque not necessarily
            // being aligned with the center of mass (e.g. when there's non-zero camber).
            auto spring_impulse = aligning_torque * dt;

            auto &row = cache.rows.emplace_back();
            row.J = {vector3_zero, normal, vector3_zero, -normal};
            row.lower_limit = std::min(spring_impulse, scalar(0));
            row.upper_limit = std::max(scalar(0), spring_impulse);
            row.inv_mA = inv_mA; row.inv_IA = inv_IA;
            row.inv_mB = inv_mB; row.inv_IB = inv_IB;
            row.dvA = &dvA; row.dwA = &dwA;
            row.dvB = &dvB; row.dwB = &dwB;
            row.impulse = imp_view.get(entity).values[0];

            auto options = constraint_row_options{};
            options.error = spring_impulse > 0 ? -large_scalar : large_scalar;

            prepare_row(row, options, linvelA, linvelB, angvelA, angvelB);
            warm_start(row);
        }

        // Aligning damping.
        {
            auto damping_impulse = aligning_damping * dt;
            auto impulse = std::abs(damping_impulse);
            // TODO: account for off-center torque application. It induces a
            // force on the center of mass as well, though only significant
            // in the presence of substantial camber.

            auto &row = cache.rows.emplace_back();
            row.J = {vector3_zero, normal, vector3_zero, -normal};
            row.lower_limit = -impulse;
            row.upper_limit =  impulse;
            row.inv_mA = inv_mA; row.inv_IA = inv_IA;
            row.inv_mB = inv_mB; row.inv_IB = inv_IB;
            row.dvA = &dvA; row.dwA = &dwA;
            row.dvB = &dvB; row.dwB = &dwB;
            row.impulse = imp_view.get(entity).values[0];

            prepare_row(row, {}, linvelA, linvelB, angvelA, angvelB);
            warm_start(row);

            con.m_aligning_damping = aligning_damping;
        }

        cache.con_num_rows.push_back(8);
    });
}

template<>
void iterate_constraints<contact_patch_constraint>(entt::registry &registry, row_cache &cache, scalar dt) {
    auto body_view = registry.view<position, orientation,
                                   linvel, angvel,
                                   mass_inv, inertia_world_inv,
                                   delta_linvel, delta_angvel>();
    auto con_view = registry.view<contact_patch_constraint>();
    auto cp_view = registry.view<contact_point>();
    auto row_idx = registry.ctx<row_start_index_contact_patch_constraint>().value;

    con_view.each([&] (entt::entity entity, contact_patch_constraint &con) {
        auto &cp = cp_view.get(entity);

        if (cp.distance > 0) {
            return;
        }

        // Adjust damping row limits to account for velocity changes during iterations.
        auto [dvA, dwA] = body_view.get<delta_linvel, delta_angvel>(con.body[0]);
        auto [dvB, dwB] = body_view.get<delta_linvel, delta_angvel>(con.body[1]);

        // Normal damping.
        {
            auto &row = cache.rows[row_idx + 1];
            auto delta_relspd = dot(row.J[0], dvA) +
                                dot(row.J[1], dwA) +
                                dot(row.J[2], dvB) +
                                dot(row.J[3], dwB);
            auto relspd = con.m_normal_relspd + delta_relspd;
            auto damping_force = con.m_normal_damping * relspd;
            auto damping_impulse = std::abs(damping_force * dt);
            row.lower_limit = 0;
            row.upper_limit = damping_impulse;
        }

        row_idx += 8;
    });

    // Longitudinal damping.
    /* {
        auto &row = registry.get<constraint_row>(con.row[3]);
        auto delta_relspd = dot(row.J[0], dvA) +
                            dot(row.J[1], dwA) +
                            dot(row.J[2], dvB) +
                            dot(row.J[3], dwB);
        auto damping_force = m_lon_damping + m_tread_damping * delta_relspd;
        auto damping_impulse = damping_force * dt;
        auto impulse = std::abs(damping_impulse);
        row.lower_limit = -impulse;
        row.upper_limit =  impulse;
    }

    // Lateral damping.
    {
        auto &row = registry.get<constraint_row>(con.row[5]);
        auto delta_relspd = dot(row.J[0], dvA) +
                            dot(row.J[1], dwA) +
                            dot(row.J[2], dvB) +
                            dot(row.J[3], dwB);
        auto damping_force = m_lat_damping + m_tread_damping * delta_relspd;
        auto damping_impulse = damping_force * dt;
        auto impulse = std::abs(damping_impulse);
        row.lower_limit = -impulse;
        row.upper_limit =  impulse;
    }

    // Aligning damping.
    {
        auto &row = registry.get<constraint_row>(con.row[7]);
        auto delta_relspd = dot(row.J[0], dvA) +
                            dot(row.J[1], dwA) +
                            dot(row.J[2], dvB) +
                            dot(row.J[3], dwB);
        auto damping_force = m_torsional_damping * relspd;
        auto damping_impulse = damping_force * dt;
        auto impulse = std::abs(damping_impulse);
        row.lower_limit = -impulse;
        row.upper_limit =  impulse;
    } */

    /*
    const auto &ornB = registry.get<orientation>(con.body[1]);
    auto &manifold = registry.get<contact_manifold>(entity);
    const auto normal = rotate(ornB, manifold.point[0].normalB);

    auto &normal_row = registry.get<constraint_row>(con.row[0]);
    auto normal_force = normal_row.impulse / dt;

    auto lon_force = scalar {0};
    auto lat_force = scalar {0};
    auto aligning_torque = scalar {0};

    for (auto &tread_row : m_tread_rows) {
        for (auto &kv : tread_row.bristles) {
            //auto tread_idx = kv.first;
            auto &bristle = kv.second;

            auto normal_pressure = tread_row.patch_half_length > 0 ?
                                  (normal_force / num_tread_rows) /
                                  (tread_row.tread_width * tread_row.patch_half_length * scalar(2 * (1 - 0.25/2 - 0.25/3))) :
                                  scalar(0);
            auto max_friction_force = bristle.friction * normal_pressure * tread_row.tread_area;
            auto dl2 = length_sqr(bristle.deflection);
            auto force = vector3_zero;

            if (dl2 > EDYN_EPSILON) {
                force = m_lon_tread_stiffness * tread_row.tread_area * bristle.deflection;

                if (length_sqr(force) > max_friction_force * max_friction_force) {
                    auto dir = bristle.deflection / std::sqrt(dl2);
                    auto max_tread_defl = bristle.friction * normal_pressure / m_lon_tread_stiffness;
                    bristle.deflection = dir * max_tread_defl;
                    force = m_lon_tread_stiffness * tread_row.tread_area * bristle.deflection;
                }
            }

            lon_force += dot(m_lon_dir, force);
            lat_force += dot(m_lat_dir, force);
            aligning_torque += dot(cross(bristle.root - m_patch_center, force), normal);
        }
    }

    // Longitudinal.
    {
        auto impulse = std::abs(lon_force * dt);
        auto &row = registry.get<constraint_row>(con.row[1]);
        row.lower_limit = -impulse;
        row.upper_limit = impulse;
    }

    // Lateral.
    {
        auto impulse = std::abs(lat_force * dt);
        auto &row = registry.get<constraint_row>(con.row[2]);
        row.lower_limit = -impulse;
        row.upper_limit = impulse;
    }

    // Aligning moment.
    {
        auto impulse = std::abs(aligning_torque * dt);
        auto &row = registry.get<constraint_row>(con.row[3]);
        row.lower_limit = -impulse;
        row.upper_limit = impulse;
    } */
}

}