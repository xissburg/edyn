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

void initialize_contact_patch_constraint(entt::registry &registry, entt::entity entity)
{
    auto &con = registry.get<contact_patch_constraint>(entity);
    auto &cp = registry.get<contact_point>(entity);
    auto [posA, ornA, spin_angleA] = registry.get<position, orientation, spin_angle>(con.body[0]);
    auto &ornB = registry.get<orientation>(con.body[1]);

    const auto axis = quaternion_x(ornA);
    const auto normal = rotate(ornB, cp.normalB);
    const auto &cyl = registry.get<cylinder_shape>(con.body[0]);
    auto axis_hl = axis * cyl.half_length;
    auto sup0 = support_point_circle(posA - axis_hl, ornA, cyl.radius, -normal);
    auto sup0_obj = to_object_space(sup0, posA, ornA);
    scalar contact_angle = std::atan2(sup0_obj.y, sup0_obj.z);

    // Transform angle from [-π, π] to [0, 2π].
    if (contact_angle < 0) {
        contact_angle += 2 * pi;
    }

    contact_angle += spin_angleA.s;
    std::fmod(contact_angle, pi2);

    con.m_contact_angle = contact_angle;
    con.m_spin_count = spin_angleA.count;
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
    auto spin_view = registry.view<spin>();

    con_view.each([&] (entt::entity entity, contact_patch_constraint &con) {
        auto &cp = cp_view.get(entity);

        auto [posA, ornA, linvelA, angvelA, inv_mA, inv_IA, dvA, dwA] =
            body_view.get<position, orientation, linvel, angvel, mass_inv, inertia_world_inv, delta_linvel, delta_angvel>(con.body[0]);
        auto [posB, ornB, linvelB, angvelB, inv_mB, inv_IB, dvB, dwB] =
            body_view.get<position, orientation, linvel, angvel, mass_inv, inertia_world_inv, delta_linvel, delta_angvel>(con.body[1]);
        auto &imp = imp_view.get(entity);

        auto spinvelA = quaternion_x(ornA) * spin_view.get(con.body[0]).s;
        auto spinvelB = vector3_zero;

        if (spin_view.contains(con.body[1])) {
            auto &s = spin_view.get(con.body[1]);
            spinvelB = quaternion_x(ornB) * scalar(s);
        }

        // Wheel spin axis in world space.
        auto axis = quaternion_x(ornA);
        auto normal = rotate(ornB, cp.normalB);
        auto &cyl = registry.get<cylinder_shape>(con.body[0]);

        // Calculate contact patch width.
        auto deflection = std::max(-cp.distance, scalar(0));
        auto sin_camber = dot(axis, normal);
        auto camber_angle = std::asin(sin_camber);
        auto normalized_contact_width = std::cos(std::atan(std::pow(std::abs(camber_angle), std::log(deflection * 300 + 1))));
        auto contact_width = cyl.half_length * 2 * normalized_contact_width;

        // Calculate center of pressure.
        auto axis_hl = axis * cyl.half_length;
        auto normalized_center_offset = -std::sin(std::atan(camber_angle));

        // Where the row starts in the x-axis in object space.
        auto row_start = sin_camber < 0 ? -cyl.half_length : cyl.half_length - contact_width;

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
        auto plane_point1 = lerp(intersection0, intersection1, (row_start + cyl.half_length + contact_width) * cyl_len_inv);
        auto center_lerp_param = (normalized_center_offset + scalar(1)) * scalar(0.5);
        auto contact_center = lerp(plane_point0, plane_point1, center_lerp_param);
        auto geometric_center = lerp(plane_point0, plane_point1, scalar(0.5));

        auto forward = cross(axis, normal);

        if (!try_normalize(forward)) {
            forward = quaternion_z(ornA);
        }

        auto tire_up = cross(forward, axis);
        auto deflection0 = std::max(dot(intersection0 - sup0, tire_up), scalar(0));
        auto deflection1 = std::max(dot(intersection1 - sup1, tire_up), scalar(0));

        // Setup non-penetration constraint.
        auto rA = contact_center - posA;
        auto rB = contact_center - posB;

        auto first_row_idx = cache.rows.size();
        auto con_row_idx = 0;

        // Normal spring.
        {
            auto linvelrel = linvelA - linvelB;
            auto speed = length(linvelrel - normal * dot(linvelrel, normal));
            auto stiffness = velocity_dependent_vertical_stiffness(con.m_normal_stiffness, speed);

            auto normal_spring_force = deflection * stiffness;
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
            row.impulse = imp.values[con_row_idx++];

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
            row.impulse = imp.values[con_row_idx++];

            prepare_row(row, {}, linvelA, linvelB, angvelA, angvelB);
            warm_start(row);
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

        auto *delta_spinA = registry.try_get<delta_spin>(con.body[0]);
        auto *delta_spinB = registry.try_get<delta_spin>(con.body[1]);

        auto tread_width = contact_width / con.num_tread_rows;
        auto r0_inv = scalar(1) / cyl.radius;

        auto &spin_angleA = registry.get<spin_angle>(con.body[0]);
        auto spin_ornA = ornA * quaternion_axis_angle(vector3_x, spin_angleA.s);
        auto &spinA = registry.get<spin>(con.body[0]);
        auto spin_angvelA = angvelA + axis * spinA;

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
        std::fmod(contact_angle, pi2);

        auto &normal_spring_row = cache.rows[first_row_idx];
        auto &normal_damping_row = cache.rows[first_row_idx + 1];
        auto normal_force = (normal_spring_row.impulse + normal_damping_row.impulse) / dt;

        // Accumulate forces and errors along all bristles.
        auto lon_force = scalar(0);
        auto lat_force = scalar(0);
        auto aligning_torque = scalar(0);
        auto lon_damping = scalar(0);
        auto lat_damping = scalar(0);
        auto aligning_damping = scalar(0);
        auto contact_len_avg = scalar(0);
        auto spin_count_delta = spin_angleA.count - con.m_spin_count;
        auto prev_contact_angle = con.m_contact_angle - spin_count_delta * pi2;
        auto bristles_per_row = con.bristles_per_row;

        for (size_t row_idx = 0; row_idx < con.num_tread_rows; ++row_idx) {
            auto &tread_row = con.m_tread_rows[row_idx];

            // The patch is divided in tread rows of equal width. Each row has a
            // different length, which is proportional to the deflection. Sample
            // the points along the middle of the row.
            auto row_x = row_start + tread_width * scalar(row_idx + 0.5);

            // Normal deflection and length for this row of bristles.
            auto row_fraction = (row_x - row_start) / contact_width;
            auto defl = lerp(deflection0, deflection1, row_fraction);

            if (defl < EDYN_EPSILON) {
                // Reset tread and bristles.
                tread_row.patch_half_length = 0;
                tread_row.prev_row_half_angle = 0;

                // Set bristle properties according to the contact angle. This is
                // an empty range of the patch thus put all bristles at the same
                // location.
                auto row_start_pos_local = vector3{
                    row_x,
                    std::sin(contact_angle) * cyl.radius,
                    std::cos(contact_angle) * cyl.radius
                };
                auto row_start_pos = project_plane(to_world_space(row_start_pos_local, posA, spin_ornA), pivotB, normal);
                tread_row.start_posB = tread_row.end_posB = to_object_space(row_start_pos, posB, ornB);

                for (auto &bristle : tread_row.bristles) {
                    bristle.tip = bristle.root = row_start_pos;
                    bristle.pivotA = row_start_pos_local;
                    bristle.pivotB = to_object_space(bristle.tip, posB, ornB);
                }

                continue;
            }

            auto row_half_length = std::min(scalar(0.4) * cyl.radius *
                                            (defl * r0_inv + scalar(2.25) *
                                            std::sqrt(defl * r0_inv)),
                                            cyl.radius * scalar(0.9));
            auto row_half_angle = std::asin(row_half_length / cyl.radius);
            auto bristle_angle_delta = (row_half_angle * 2) / scalar(bristles_per_row);
            auto prev_bristle_angle_delta = (tread_row.prev_row_half_angle * 2) / scalar(bristles_per_row);

            contact_len_avg += row_half_length * 2;

            // Contact patch extents in radians for this row.
            auto patch_start_angle = contact_angle - row_half_angle;
            auto patch_end_angle   = contact_angle + row_half_angle;

            // Previous contact patch range accounting for the number of spins
            // since the last update so that the ranges and angles are all laid
            // out in a segment without wrapping around.
            auto prev_patch_start_angle = prev_contact_angle - tread_row.prev_row_half_angle;
            auto prev_patch_end_angle   = prev_contact_angle + tread_row.prev_row_half_angle;

            // Calculate intersection between previous and current contact patch
            // range. Bristles that lie in the intersection will be matched to a
            // brush tip calculated as an interpolation of the previous values.
            // For the bristles that lie outside of the intersection, assign a
            // brush tip from the line connecting the start of the previous and
            // current contact patches for this row.
            auto intersection_start_angle = std::max(patch_start_angle, prev_patch_start_angle);
            auto intersection_end_angle = std::min(patch_end_angle, prev_patch_end_angle);
            auto intersects = intersection_start_angle < intersection_end_angle;

            auto row_start_pos_local = vector3{row_x,
                                               std::sin(patch_start_angle) * cyl.radius,
                                               std::cos(patch_start_angle) * cyl.radius};
            auto row_end_pos_local   = vector3{row_x,
                                               std::sin(patch_end_angle) * cyl.radius,
                                               std::cos(patch_end_angle) * cyl.radius};
            auto row_start_pos = project_plane(to_world_space(row_start_pos_local, posA, spin_ornA), pivotB, normal);
            auto row_end_pos   = project_plane(to_world_space(row_end_pos_local, posA, spin_ornA), pivotB, normal);

            auto prev_row_start_pos = to_world_space(tread_row.start_posB, posB, ornB);
            auto prev_row_end_pos   = to_world_space(tread_row.end_posB, posB, ornB);

            auto prev_bristle_defl = vector3_zero;
            auto bristle_angle_start = patch_start_angle + bristle_angle_delta * scalar(0.5);

            for (size_t bristle_idx = 0; bristle_idx < bristles_per_row; ++bristle_idx) {
                auto &bristle = tread_row.bristles[bristle_idx];
                auto bristle_angle = bristle_angle_start + bristle_angle_delta * scalar(bristle_idx);
                auto bristle_tip = vector3_zero;

                if (intersects && bristle_angle >= intersection_start_angle && bristle_angle <= intersection_end_angle) {
                    // Bristle lies in the intersection.
                    // Find index of bristles in the previous patch which surround this bristle.
                    auto fraction = (bristle_angle - prev_patch_start_angle) /
                                    (prev_patch_end_angle - prev_patch_start_angle);
                    auto after_idx = static_cast<size_t>(std::round(fraction * bristles_per_row));
                    EDYN_ASSERT(after_idx <= bristles_per_row);
                    scalar before_angle, after_angle;
                    vector3 before_pivotB, after_pivotB;

                    if (after_idx == 0) {
                        // This bristle is located before the first bristle in the previous
                        // contact patch. Use the start of the previous patch as the location
                        // before it and the first bristle as the location after it.
                        before_angle = prev_patch_start_angle;
                        after_angle = before_angle + prev_bristle_angle_delta * scalar(0.5);
                        before_pivotB = tread_row.start_posB;
                        after_pivotB = tread_row.bristles[after_idx].pivotB;
                    } else if (after_idx == bristles_per_row) {
                        after_angle = prev_patch_end_angle;
                        before_angle = after_angle - prev_bristle_angle_delta * scalar(0.5);
                        before_pivotB = tread_row.bristles[after_idx - 1].pivotB;
                        after_pivotB = tread_row.end_posB;
                    } else {
                        auto before_idx = after_idx - 1;
                        before_angle = prev_patch_start_angle + prev_bristle_angle_delta * scalar(before_idx + 0.5);
                        after_angle = prev_patch_start_angle + prev_bristle_angle_delta * scalar(after_idx + 0.5);
                        before_pivotB = tread_row.bristles[before_idx].pivotB;
                        after_pivotB = tread_row.bristles[after_idx].pivotB;
                    }

                    // Linearly interpolate the tips.
                    auto inbetween_fraction = (bristle_angle - before_angle) / (after_angle - before_angle);
                    bristle.pivotB = lerp(before_pivotB, after_pivotB, inbetween_fraction);
                    bristle_tip = to_world_space(bristle.pivotB, posB, ornB);
                } else if (bristle_angle >= prev_patch_end_angle) {
                    auto fraction = (bristle_angle - prev_patch_end_angle) /
                                    (patch_end_angle - prev_patch_end_angle);
                    bristle_tip = lerp(prev_row_end_pos, row_end_pos, fraction);
                    bristle.pivotB = to_object_space(bristle_tip, posB, ornB);
                } else if (bristle_angle <= prev_patch_start_angle) {
                    auto fraction = (bristle_angle - prev_patch_start_angle) /
                                    (patch_start_angle - prev_patch_start_angle);
                    bristle_tip = lerp(prev_row_start_pos, row_start_pos, fraction);
                    bristle.pivotB = to_object_space(bristle_tip, posB, ornB);
                } else {
                    EDYN_ASSERT(false);
                }

                auto bristle_root_local = vector3{row_x,
                                                  std::sin(bristle_angle) * cyl.radius,
                                                  std::cos(bristle_angle) * cyl.radius};
                auto bristle_root = project_plane(to_world_space(bristle_root_local, posA, spin_ornA), pivotB, normal);
                bristle.pivotA = bristle_root_local;

                auto normal_pressure = (normal_force / con.num_tread_rows) / (tread_width * 2 * row_half_length);
                auto mu0 = cp.friction * std::exp(scalar(-0.001) * con.m_load_sensitivity * normal_force);
                bristle.friction = mu0 / (1 + con.m_speed_sensitivity * bristle.sliding_spd);

                // The length for the first bristle is halved since it is located in
                // the middle of the rectangular tread.
                auto tread_length = (bristle_idx == 0 ? scalar(0.5) : scalar(1)) * scalar(2) * row_half_length / scalar(bristles_per_row);
                auto tread_area = tread_width * tread_length;


                // TODO: handle anysotropic stiffness.
                // The force is calculated as an integral from the previous deflection until the
                // current along the row.
                auto bristle_defl = bristle_root - bristle_tip;
                auto spring_force = con.m_lon_tread_stiffness * tread_area *
                                    (prev_bristle_defl + bristle_defl) * scalar(0.5);
                auto max_friction_force = bristle.friction * normal_pressure * tread_area;

                // Bristle deflection force is greater than maximum friction force
                // for the current normal load, which means the bristle must slide.
                // Thus, move the bristle tip closer to its root so that the
                // tangential deflection force is equals to the friction force.
                if (length_sqr(spring_force) > max_friction_force * max_friction_force) {
                    auto error = length(bristle_defl); EDYN_ASSERT(error > 0);
                    auto dir = bristle_defl / error;
                    auto max_tread_defl = bristle.friction * normal_pressure / con.m_lon_tread_stiffness;
                    bristle_defl = dir * max_tread_defl;
                    spring_force = con.m_lon_tread_stiffness * tread_area *
                                    (prev_bristle_defl + bristle_defl) * scalar(0.5);
                    bristle_tip = bristle_root - bristle_defl;

                    // Move pivot in B to match new tip location.
                    bristle.pivotB = to_object_space(bristle_tip, posB, ornB);
                }

                // Calculate bristle tip velocity by subtracting current from previous
                // position and dividing by the elapsed time.
                auto tip_vel = project_direction((bristle_tip - bristle.tip) / dt, normal);
                auto sliding_spd = length(tip_vel);
                bristle.sliding_spd = sliding_spd;

                // Point of force application.
                auto prev_bristle_root = bristle_idx > 0 ?
                    tread_row.bristles[bristle_idx - 1].root :
                    row_start_pos;
                auto midpoint = (bristle_root + prev_bristle_root) * scalar(0.5);

                auto root_vel = project_direction(linvelA + cross(spin_angvelA, bristle_root - posA), normal);
                auto bristle_defl_vel = tip_vel - root_vel;

                auto damping_force = bristle_defl_vel * con.m_tread_damping * tread_area;
                lon_damping += dot(con.m_lon_dir, damping_force);
                lat_damping += dot(con.m_lat_dir, damping_force);
                aligning_damping += dot(cross(midpoint - contact_center, damping_force), normal);

                lon_force += dot(con.m_lon_dir, -spring_force);
                lat_force += dot(con.m_lat_dir, -spring_force);
                aligning_torque += dot(cross(midpoint - contact_center, -spring_force), normal);

                bristle.deflection = bristle_defl;
                bristle.tip = bristle_tip;
                bristle.root = bristle_root;

                prev_bristle_defl = bristle_defl;
            }

            // Add force from last bristle until end of patch.
            {
                auto tread_length = scalar(0.5) * scalar(2) * row_half_length / scalar(bristles_per_row);
                auto tread_area = tread_width * tread_length;
                auto spring_force = con.m_lon_tread_stiffness * tread_area * prev_bristle_defl * scalar(0.5);
                lon_force += dot(con.m_lon_dir, -spring_force);
                lat_force += dot(con.m_lat_dir, -spring_force);
                auto bristle_root = tread_row.bristles.back().root;
                auto midpoint = (row_end_pos + bristle_root) * scalar(0.5);
                aligning_torque += dot(cross(midpoint - contact_center, -spring_force), normal);
            }

            tread_row.prev_row_half_angle = row_half_angle;
            tread_row.patch_half_length = row_half_length;
            tread_row.start_posB = to_object_space(row_start_pos, posB, ornB);
            tread_row.end_posB = to_object_space(row_end_pos, posB, ornB);
        }

        contact_len_avg /= con.num_tread_rows;

        size_t total_bristles = 0;
        con.m_sliding_spd_avg = scalar {0};

        for (auto &row : con.m_tread_rows) {
            for (auto &bristle : row.bristles) {
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
            row.dvA = &dvA; row.dwA = &dwA; row.dsA = delta_spinA;
            row.dvB = &dvB; row.dwB = &dwB; row.dsB = delta_spinB;
            row.impulse = imp.values[con_row_idx++];
            row.use_spin[0] = true;
            row.use_spin[1] = true;
            row.spin_axis[0] = axis;
            row.spin_axis[1] = quaternion_x(ornB);

            auto options = constraint_row_options{};
            options.error = spring_impulse > 0 ? -large_scalar : large_scalar;

            prepare_row(row, options, linvelA, linvelB, angvelA + spinvelA, angvelB + spinvelB);
            warm_start(row);
        }

        // Longitudinal damping.
        /* {
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
            row.dvA = &dvA; row.dwA = &dwA; row.dsA = delta_spinA;
            row.dvB = &dvB; row.dwB = &dwB; row.dsB = delta_spinB;
            row.impulse = imp.values[con_row_idx++];
            row.use_spin[0] = true;
            row.use_spin[1] = true;
            row.spin_axis[0] = axis;
            row.spin_axis[1] = quaternion_x(ornB);

            prepare_row(row, {}, linvelA, linvelB, angvelA + spinvelA, angvelB + spinvelB);
            warm_start(row);

            con.m_lon_damping = lon_damping;
        } */

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
            row.impulse = imp.values[con_row_idx++];

            auto options = constraint_row_options{};
            options.error = spring_impulse > 0 ? -large_scalar : large_scalar;

            prepare_row(row, options, linvelA, linvelB, angvelA, angvelB);
            warm_start(row);
        }

        // Lateral damping.
        /* {
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
            row.impulse = imp.values[con_row_idx++];

            prepare_row(row, {}, linvelA, linvelB, angvelA, angvelB);
            warm_start(row);

            con.m_lat_damping = lat_damping;
        } */

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
            row.impulse = imp.values[con_row_idx++];

            auto options = constraint_row_options{};
            options.error = spring_impulse > 0 ? -large_scalar : large_scalar;

            prepare_row(row, options, linvelA, linvelB, angvelA, angvelB);
            warm_start(row);
        }

        // Aligning damping.
        /* {
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
            row.impulse = imp.values[con_row_idx++];

            prepare_row(row, {}, linvelA, linvelB, angvelA, angvelB);
            warm_start(row);

            con.m_aligning_damping = aligning_damping;
        } */

        con.m_sin_camber = sin_camber;
        con.m_contact_width = contact_width;
        con.m_contact_len_avg = contact_len_avg;
        con.m_center = geometric_center;
        con.m_pivot = contact_center;
        con.m_normal = normal;
        con.m_deflection = deflection;
        con.m_contact_angle = contact_angle;
        con.m_spin_count = spin_angleA.count;

        cache.con_num_rows.push_back(con_row_idx);
    });
}

}