#include "edyn/constraints/contact_patch_constraint.hpp"
#include "edyn/comp/contact_manifold.hpp"
#include "edyn/comp/constraint.hpp"
#include "edyn/comp/constraint_row.hpp"
#include "edyn/comp/relation.hpp"
#include "edyn/comp/shape.hpp"
#include "edyn/comp/position.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/comp/linvel.hpp"
#include "edyn/comp/angvel.hpp"
#include "edyn/comp/delta_linvel.hpp"
#include "edyn/comp/delta_angvel.hpp"
#include "edyn/comp/spin.hpp"
#include "edyn/util/tire.hpp"
#include "edyn/math/matrix3x3.hpp"
#include <entt/entt.hpp>

namespace edyn {

static
vector3 support_point_circle(const vector3 &pos, const quaternion &orn, scalar radius, const vector3 &dir) {
    auto normal = rotate(orn, vector3_x);
    auto proj = dir - normal * dot(dir, normal);
    auto l2 = length2(proj);

    if (l2 > EDYN_EPSILON) {
        auto s = radius / std::sqrt(l2);
        return pos + proj * s;
    }

    return pos + rotate(orn, vector3_y * radius);
}

void contact_patch_constraint::clear(entt::registry &registry, constraint &con) {
    for (size_t i = 0; i < con.num_rows; ++i) {
        registry.destroy(con.row[i]);
        con.row[i] = entt::null;
    }

    con.num_rows = 0;

    for (size_t i = 0; i < num_tread_rows; ++i) {
        auto &tread_row = m_tread_rows[i];
        tread_row.bristles.clear();
    }
}

void contact_patch_constraint::prepare(entt::entity entity, constraint &con, 
                                       const relation &rel, entt::registry &registry,
                                       scalar dt) {
    auto &manifold = registry.get<contact_manifold>(entity);
    
    if (manifold.num_points == 0) {
        clear(registry, con);
        return;
    }

    // Use largest penetration for normal deflection force calculation.
    auto deepest_distance = EDYN_SCALAR_MAX;
    auto pt_idx = size_t {0};

    for (size_t i = 0; i < manifold.num_points; ++i) {
        if (manifold.point[i].distance < deepest_distance) {
            deepest_distance = manifold.point[i].distance;
            pt_idx = i;
        }
    }

    if (deepest_distance > 0) {
        clear(registry, con);
        return;
    }

    const auto &posA = registry.get<position>(rel.entity[0]);
    const auto &ornA = registry.get<orientation>(rel.entity[0]);
    
    // Wheel spin axis in world space.
    const auto axis = rotate(ornA, vector3_x);
    
    const auto &spin_angleA = registry.get<spin_angle>(rel.entity[0]);
    const auto spin_ornA = ornA * quaternion_axis_angle(vector3_x, spin_angleA);

    const auto &posB = registry.get<position>(rel.entity[1]);
    const auto &ornB = registry.get<orientation>(rel.entity[1]);
    const auto ornB_conj = conjugate(ornB);
    
    const auto &linvelA = registry.get<linvel>(rel.entity[0]);
    const auto &angvelA = registry.get<angvel>(rel.entity[0]);
    
    const auto &spinA = registry.get<spin>(rel.entity[0]);
    const auto spin_angvelA = angvelA + axis * spinA;

    const auto &linvelB = registry.get<linvel>(rel.entity[1]);
    const auto &angvelB = registry.get<angvel>(rel.entity[1]);

    const auto &shapeA = registry.get<shape>(rel.entity[0]);
    const auto &cyl = std::get<cylinder_shape>(shapeA.var);
    
    const auto normal = rotate(ornB, manifold.point[pt_idx].normalB);

    // Determine contact patch extremities.
    const auto axis_hl = axis * cyl.half_length;
    auto p0 = support_point_circle(posA + axis_hl, ornA, cyl.radius, -normal);
    auto p1 = p0 - axis_hl * 2; // because circles are parallel

    // A point on the contact plane.
    auto pB = posB + rotate(ornB, manifold.point[pt_idx].pivotB);

    auto proj0 = dot(p0 - pB, normal);
    auto proj1 = dot(p1 - pB, normal);

    // Where the row starts in the x-axis in object space.
    auto row_start = -cyl.half_length;
    
    // Intersect segment between `p0` and `p1` with contact plane to find
    // the extreme points along the width of the contact patch.
    auto d = dot(axis_hl, normal);

    if (std::abs(d) > EDYN_EPSILON) {
        if (proj0 > 0) {
            // `p0` is above the plane.
            p0 -= axis_hl * proj0 / d;
        } else if (proj1 > 0) {
            // `p1` is above the plane.
            auto s = proj1 / d;
            p1 -= axis_hl * s;
            row_start -= cyl.half_length * s;
        }
    }

    // Create constraint rows if needed.
    if (con.num_rows == 0) {
        // A spring row and a damper row for normal, longitudinal, lateral and 
        // torsional directions.
        con.num_rows = 8;

        for (size_t i = 0; i < con.num_rows; ++i) {
            auto [row_entity, row] = registry.create<constraint_row>();
            row.entity = rel.entity;
            // Priority zero for normal constraint,
            // priority one for friction constraints.
            row.priority = i == 0 || i == 1 ? 0 : 1;
            // Use spin only for longitudinal constraint.
            row.use_spin[0] = i == 2 || i == 3;
            row.use_spin[1] = i == 2 || i == 3;
            con.row[i] = row_entity;
        }
    }

    // The center of the contact patch on the contact plane.
    auto tire_patch_center = (p0 + p1) * 0.5;
    auto patch_center_on_axis = posA + axis * dot(tire_patch_center - posA, axis);
    auto patch_center_dir = tire_patch_center - patch_center_on_axis;
    auto patch_center_param = dot(pB - patch_center_on_axis, normal) / dot(patch_center_dir, normal);
    m_patch_center = posA + patch_center_dir * patch_center_param;

    // Setup non-penetration constraint.
    auto rA = m_patch_center - posA;
    auto rB = m_patch_center - posB;

    auto tire_rA = tire_patch_center - posA;
    
    auto &normal_spring_row = registry.get<constraint_row>(con.row[0]);
    normal_spring_row.J = {normal, cross(rA, normal), -normal, -cross(rB, normal)};
    
    auto &normal_damping_row = registry.get<constraint_row>(con.row[1]);
    normal_damping_row.J = {normal, cross(rA, normal), -normal, -cross(rB, normal)};

    // Normal spring.
    {
        auto linvelrel = linvelA - linvelB;
        auto speed = length(linvelrel - normal * dot(linvelrel, normal));
        auto stiffness = velocity_dependent_vertical_stiffness(m_normal_stiffness, speed);
        auto penetration = dot(posA + tire_rA - posB - rB, normal);

        auto normal_spring_force = -penetration * stiffness;
        auto normal_spring_impulse = normal_spring_force * dt;
        normal_spring_row.error = normal_spring_impulse > 0 ? -large_scalar : large_scalar;
        normal_spring_row.lower_limit = 0;
        normal_spring_row.upper_limit = std::max(scalar(0), normal_spring_impulse);
    }

    // Normal damping.
    {
        auto normal_relspd = dot(normal_damping_row.J[0], linvelA) + 
                             dot(normal_damping_row.J[1], angvelA) +
                             dot(normal_damping_row.J[2], linvelB) +
                             dot(normal_damping_row.J[3], angvelB);
        auto normal_damper_force = m_normal_damping * normal_relspd;
        auto normal_damper_impulse = std::abs(normal_damper_force * dt);

        normal_damping_row.lower_limit = 0;
        normal_damping_row.upper_limit = normal_damper_impulse;
        normal_damping_row.error = 0;

        m_normal_relspd = normal_relspd;
    }

    auto forward = normalize(cross(axis, normal));
    auto up = cross(forward, axis);

    // Calculate longitudinal and lateral friction directions.
    m_lat_dir = axis - normal * dot(axis, normal);
    auto lat_dir_len2 = length2(m_lat_dir);

    if (lat_dir_len2 > EDYN_EPSILON) {
        m_lat_dir /= std::sqrt(lat_dir_len2);
        m_lon_dir = cross(m_lat_dir, normal);
    } else {
        auto tire_z = rotate(ornA, vector3_z);
        m_lon_dir = tire_z - normal * dot(tire_z, normal);
        m_lon_dir = normalize(m_lon_dir);
        m_lat_dir = cross(normal, m_lon_dir);
    }

    auto contact_width = length(p0 - p1);
    auto tread_width = contact_width / num_tread_rows;
    auto r0_inv = scalar(1) / cyl.radius;

    const scalar half_perimeter = pi * cyl.radius;
    const scalar perimeter = half_perimeter * 2;
    const scalar desired_spacing = 0.01;
    const uint16_t num_bristles = std::floor(perimeter / desired_spacing);
    const scalar bristle_spacing = perimeter / num_bristles;
    const scalar bristle_angle_delta = bristle_spacing * r0_inv;

    // Support point in object space.
    auto p0_obj = rotate(conjugate(ornA), p0 - posA);

    // Support point angle in circle space where z points forward and y is up.
    // It allows us to observe the contact point location with respect to the
    // current spin angle and determine which bristles lie within the contact
    // patch range.
    scalar contact_angle = std::atan2(p0_obj.y, p0_obj.z);

    // Transform angle from [-π, π] to [0, 2π].
    if (contact_angle < 0) {
        contact_angle += 2 * pi;
    }

    contact_angle += spin_angleA.s;

    auto normal_force = (normal_spring_row.impulse + normal_damping_row.impulse) / dt;

    // Accumulate forces and errors along all bristles.
    auto lon_force = scalar {0};
    auto lat_force = scalar {0};
    auto aligning_torque = scalar {0};
    auto lon_damping = scalar {0};
    auto lat_damping = scalar {0};
    auto aligning_damping = scalar {0};
    auto lon_error = scalar {0};
    auto lat_error = scalar {0};
    auto aligning_error = scalar {0};
    auto total_bristles = uint16_t {0};

    // Update bristles for each row.
    for (size_t i = 0; i < num_tread_rows; ++i) {
        auto &tread_row = m_tread_rows[i];
        auto row_x = row_start + tread_width * scalar(i + 0.5);
        auto row_center_cyl = p0 - axis_hl + axis * row_x;

        // Normal deflection and length for this row of bristles.
        const auto defl = std::clamp(dot(row_center_cyl - pB, -normal), scalar(0), cyl.radius / 2);
        const auto row_half_length = scalar(0.4) * cyl.radius * (defl * r0_inv + scalar(2.25) * std::sqrt(defl * r0_inv));
        const auto row_half_angle = std::asin(row_half_length / cyl.radius);

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
            ++total_bristles;
            const auto bristle_idx = (start_idx + j) % num_bristles;
            brush_bristle *bristle;

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
                auto entry_bristle_pos = project_plane(entry_posA + rotate(entry_spin_ornA, bristle_pivot), pB, normal);

                // Calculate pivots at time of entry.
                auto entry_posB = posB + linvelB * entry_dt;
                auto entry_ornB = integrate(ornB, angvelB, entry_dt);
                auto rB = rotate(conjugate(entry_ornB), entry_bristle_pos - entry_posB);
                auto rA = bristle_pivot;

                tread_row.bristles[bristle_idx] = brush_bristle {rA, rB};
                bristle = &tread_row.bristles[bristle_idx];

                // Set tip position to current.
                bristle->tip = project_plane(posB + rotate(ornB, rB), pB, normal);
            }

            // Bristle root and tip in world space, on the contact plane.
            auto bristle_root = project_plane(posA + rotate(spin_ornA, bristle->pivotA), pB, normal);
            auto bristle_tip = project_plane(posB + rotate(ornB, bristle->pivotB), pB, normal);

            auto normal_pressure = row_half_length > 0 ? 
                                   (normal_force / num_tread_rows) / 
                                   (tread_width * 2 * row_half_length) : 
                                   scalar(0);
            auto mu0 = m_friction_coefficient * std::exp(scalar(-0.001) * m_load_sensitivity * normal_force);
            bristle->friction = mu0 / (1 + m_speed_sensitivity * bristle->sliding_spd);

            auto spring_force = vector3_zero;
            auto bristle_defl = bristle_root - bristle_tip;
            auto bristle_defl_len2 = length2(bristle_defl);

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
                spring_force = m_lon_tread_stiffness * tread_width * 
                                (prev_bristle_defl * dx + (bristle_defl - prev_bristle_defl) * dx * scalar(0.5));

                // The area of the first tread is a proportion of the total.
                auto local_area = j > 0 ? tread_area :
                                  scalar(0.5) * tread_area * (a1 - a0) / bristle_angle_delta;
                auto max_friction_force = bristle->friction * normal_pressure * local_area;

                // Bristle deflection force is greater than maximum friction force
                // for the current normal load, which mean the bristle must slide.
                // Thus, move the bristle tip closer to its root so that the 
                // tangential deflection force is equals to the friction force.
                if (length2(spring_force) > max_friction_force * max_friction_force) {
                    auto error = std::sqrt(bristle_defl_len2);
                    auto dir = bristle_defl / error;
                    auto max_tread_defl = bristle->friction * normal_pressure / m_lon_tread_stiffness;
                    bristle_defl = dir * max_tread_defl;
                    spring_force = m_lon_tread_stiffness * tread_width * 
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

            auto damping_force = bristle_defl_vel * m_tread_damping * tread_area;
            lon_damping += dot(m_lon_dir, damping_force);
            lat_damping += dot(m_lat_dir, damping_force);
            aligning_damping += dot(cross(bristle_root - m_patch_center, damping_force), normal);

            lon_force += dot(m_lon_dir, -spring_force);
            lat_force += dot(m_lat_dir, -spring_force);
            aligning_torque += dot(cross(bristle_root - m_patch_center, -spring_force), normal);

            lon_error += dot(m_lon_dir, bristle_defl);
            lat_error += dot(m_lat_dir, bristle_defl);

            // Angle (approx.) subtended by bristle from the patch center.
            aligning_error += dot(cross(bristle_root - m_patch_center, bristle_tip - m_patch_center), normal);

            bristle->deflection = bristle_defl;
            bristle->tip = bristle_tip;
            bristle->root = bristle_root;

            prev_bristle_defl = bristle_defl;
        
            // Add force from last bristle until end of patch.
            if (j == row_num_bristles - 1) {
                auto x0 = (start_angle + scalar(j) * bristle_angle_delta) * cyl.radius;
                auto x1 = patch_end_angle * cyl.radius;
                auto dx = x1 - x0;

                spring_force = m_lon_tread_stiffness * tread_width * 
                                (prev_bristle_defl * dx + (bristle_defl - prev_bristle_defl) * dx * scalar(0.5));
                lon_force += dot(m_lon_dir, -spring_force);
                lat_force += dot(m_lat_dir, -spring_force);
                aligning_torque += dot(cross(bristle_root - m_patch_center, -spring_force), normal);
            }
        }

        tread_row.tread_area = tread_area;
        tread_row.prev_contact_angle = contact_angle;
        tread_row.prev_spin_count = spin_angleA.count;
        tread_row.prev_row_half_angle = row_half_angle;
    }

    if (total_bristles > 0) {
        lon_error /= total_bristles;
        lat_error /= total_bristles;
        aligning_error /= total_bristles;
    }

    // Longitudinal stiffness.
    {
        auto p = cross(rA, m_lon_dir);
        auto q = cross(rB, m_lon_dir);
        auto spring_impulse = lon_force * dt;

        auto &row = registry.get<constraint_row>(con.row[2]);
        row.J = {m_lon_dir, p, -m_lon_dir, -q};
        row.error = spring_impulse > 0 ? -large_scalar : large_scalar;
        row.lower_limit = std::min(spring_impulse, scalar(0));
        row.upper_limit = std::max(scalar(0), spring_impulse);
    }

    // Longitudinal damping.
    {
        auto p = cross(rA, m_lon_dir);
        auto q = cross(rB, m_lon_dir);
        auto damping_impulse = lon_damping * dt;
        auto impulse = std::abs(damping_impulse);

        auto &row = registry.get<constraint_row>(con.row[3]);
        row.J = {m_lon_dir, p, -m_lon_dir, -q};
        row.error = 0;
        row.lower_limit = -impulse;
        row.upper_limit =  impulse;

        m_lon_damping = lat_damping;
    }

    // Lateral stiffness.
    {
        auto p = cross(rA, m_lat_dir);
        auto q = cross(rB, m_lat_dir);
        auto spring_impulse = lat_force * dt;

        auto &row = registry.get<constraint_row>(con.row[4]);
        row.J = {m_lat_dir, p, -m_lat_dir, -q};
        row.error = spring_impulse > 0 ? -large_scalar : large_scalar;
        row.lower_limit = std::min(spring_impulse, scalar(0));
        row.upper_limit = std::max(scalar(0), spring_impulse);
    }

    // Lateral damping.
    {
        auto p = cross(rA, m_lat_dir);
        auto q = cross(rB, m_lat_dir);
        auto damping_impulse = lat_damping * dt;
        auto impulse = std::abs(damping_impulse);

        auto &row = registry.get<constraint_row>(con.row[5]);
        row.J = {m_lat_dir, p, -m_lat_dir, -q};
        row.error = 0;
        row.lower_limit = -impulse;
        row.upper_limit =  impulse;

        m_lat_damping = lat_damping;
    }

    // Aligning moment.
    {
        // TODO: Also apply force on center of mass due to torque not necessarily
        // being aligned with the center of mass (e.g. when there's non-zero camber).
        auto spring_impulse = aligning_torque * dt;

        auto &row = registry.get<constraint_row>(con.row[6]);
        row.J = {vector3_zero, normal, vector3_zero, -normal};
        row.error = spring_impulse > 0 ? -large_scalar : large_scalar;
        row.lower_limit = std::min(spring_impulse, scalar(0));
        row.upper_limit = std::max(scalar(0), spring_impulse);
    }

    // Aligning damping.
    {
        auto damping_impulse = aligning_damping * dt;
        auto impulse = std::abs(damping_impulse);

        auto &row = registry.get<constraint_row>(con.row[7]);
        row.J = {vector3_zero, normal, vector3_zero, -normal};
        row.error = 0;
        row.lower_limit = -impulse;
        row.upper_limit =  impulse;

        m_aligning_damping = aligning_damping;
    }
}

void contact_patch_constraint::iteration(entt::entity entity, constraint &con, 
                                         const relation &rel, entt::registry &registry, 
                                         scalar dt) {
    if (con.num_rows == 0) {
        return;
    }

    // Adjust damping row limits to account for velocity changes during iterations.
    auto &dvA = registry.get<delta_linvel>(rel.entity[0]);
    auto &dwA = registry.get<delta_angvel>(rel.entity[0]);
    auto &dvB = registry.get<delta_linvel>(rel.entity[1]);
    auto &dwB = registry.get<delta_angvel>(rel.entity[1]);

    // Normal damping.
    {
        auto &row = registry.get<constraint_row>(con.row[1]);
        auto delta_relspd = dot(row.J[0], dvA) + 
                            dot(row.J[1], dwA) +
                            dot(row.J[2], dvB) +
                            dot(row.J[3], dwB);
        auto relspd = m_normal_relspd + delta_relspd;
        auto damping_force = m_normal_damping * relspd;
        auto damping_impulse = std::abs(damping_force * dt);
        row.lower_limit = 0;
        row.upper_limit = damping_impulse;
    }

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
    const auto &ornB = registry.get<orientation>(rel.entity[1]);
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
            auto dl2 = length2(bristle.deflection);
            auto force = vector3_zero;

            if (dl2 > EDYN_EPSILON) {
                force = m_lon_tread_stiffness * tread_row.tread_area * bristle.deflection;

                if (length2(force) > max_friction_force * max_friction_force) {
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