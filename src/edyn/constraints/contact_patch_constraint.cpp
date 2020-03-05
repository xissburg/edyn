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
#include "edyn/comp/spin.hpp"
#include "edyn/math/matrix3x3.hpp"
#include <entt/entt.hpp>

#include <iostream>

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

    auto deepest_distance = EDYN_SCALAR_MAX;
    auto pt_idx = size_t {0};
    for (size_t i = 0; i < manifold.num_points; ++i) {
        if (manifold.point[i].distance < deepest_distance) {
            deepest_distance = manifold.point[i].distance;
            pt_idx = i;
        }
    }

    const auto &posA = registry.get<position>(rel.entity[0]);
    const auto &ornA = registry.get<orientation>(rel.entity[0]);
    
    const auto axis = rotate(ornA, vector3_x);
    
    const auto &spin_angleA = registry.get<spin_angle>(rel.entity[0]);
    const auto spin_ornA = ornA * quaternion_axis_angle(vector3_x, spin_angleA);

    const auto &posB = registry.get<position>(rel.entity[1]);
    const auto &ornB = registry.get<orientation>(rel.entity[1]);
    const auto ornB_conj = conjugate(ornB);

    const auto normal = rotate(ornB, manifold.point[pt_idx].normalB);

    const auto &shapeA = registry.get<shape>(rel.entity[0]);
    const auto &cyl = std::get<cylinder_shape>(shapeA.var);
    
    const auto &linvelA = registry.get<linvel>(rel.entity[0]);
    const auto &angvelA = registry.get<angvel>(rel.entity[0]);
    
    const auto &spinA = registry.get<spin>(rel.entity[0]);
    const auto spin_angvelA = angvelA + axis * spinA;

    const auto &linvelB = registry.get<linvel>(rel.entity[1]);
    const auto &angvelB = registry.get<angvel>(rel.entity[1]);

    // Determine contact patch extremities.
    const auto axis_hl = axis * cyl.half_length;
    auto p0 = support_point_circle(posA + axis_hl, ornA, cyl.radius, -normal);
    auto p1 = p0 - axis_hl * 2; // because circles are parallel

    // A point on the contact plane.
    auto pB = posB + rotate(ornB, manifold.point[pt_idx].pivotB);

    auto proj0 = dot(p0 - pB, normal);
    auto proj1 = dot(p1 - pB, normal);

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
            p1 += axis_hl * s;
            row_start += cyl.half_length * s;
        }
    }

    // Create constraint rows if needed.
    if (con.num_rows == 0) {
        con.num_rows = 4;

        for (size_t i = 0; i < con.num_rows; ++i) {
            auto [row_entity, row] = registry.create<constraint_row>();
            row.entity = rel.entity;
            // Priority zero for normal constraint,
            // priority one for friction constraints.
            row.priority = i == 0 ? 0 : 1;
            // Use spin for longitudinal constraint.
            row.use_spin[0] = i == 1;
            row.use_spin[1] = i == 1;
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
    
    auto &normal_row = registry.get<constraint_row>(con.row[0]);
    normal_row.J = {normal, cross(rA, normal), -normal, -cross(rB, normal)};
    normal_row.lower_limit = 0;

    auto linvelrel = linvelA - linvelB;
    auto speed = length(linvelrel - normal * dot(linvelrel, normal));
    auto stiffness = velocity_dependent_vertical_stiffness(m_stiffness, speed);
    auto penetration = dot(posA + tire_rA - posB - rB, normal);
    auto spring_force = penetration * stiffness;
    auto normal_relvel = dot(linvelA + cross(angvelA, rA) - linvelB - cross(angvelB, rB), normal);
    auto damper_force = normal_relvel * m_damping;
    normal_row.upper_limit = std::abs(spring_force + damper_force) * dt;

    auto pvel = penetration / dt;

    normal_row.error = 0;
    normal_row.restitution = 0;

    // If not penetrating and the velocity necessary to touch in `dt` seconds
    // is smaller than the bounce velocity, it should apply an impulse that
    // will prevent penetration after the following physics update.
    if (penetration > 0) {
        normal_row.error = std::max(pvel, scalar(0));
    } else {
        // If this is a resting contact and it is penetrating, apply impulse to push it out.
        //if (cp.lifetime > 0) {
            constexpr scalar contact_erp = 0.2;
            normal_row.error = std::min(pvel, scalar(0)) * contact_erp;
        //}
    }

    auto forward = normalize(cross(axis, normal));
    auto up = cross(forward, axis);

    // Calculate longitudinal and lateral friction directions
    // project axis on contact plane.
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

    contact_angle += spin_angleA;

    std::cout << "================================================" << std::endl;
    std::cout << "contact_angle: " << contact_angle << std::endl;

    auto normal_force = normal_row.impulse / dt;
    auto lon_force = scalar {0};
    auto lat_force = scalar {0};
    auto aligning_torque = scalar {0};
    auto lon_error = scalar {0};
    auto lat_error = scalar {0};
    auto aligning_error = scalar {0};
    auto total_bristles = uint16_t {0};

    // Update bristles for each row.
    for (size_t i = 0; i < num_tread_rows; ++i) {
        auto &tread_row = m_tread_rows[i];
        auto row_x = row_start + tread_width * (i + 0.5);
        auto row_center_cyl = p0 - axis_hl + axis * row_x;
        auto defl = std::clamp(dot(row_center_cyl - pB, -normal), scalar(0), cyl.radius / 2);
        auto row_half_length = scalar(0.4) * cyl.radius * (defl * r0_inv + scalar(2.25) * std::sqrt(defl * r0_inv));

        const auto row_half_angle = std::asin(row_half_length / cyl.radius);
        const auto row_num_bristles = std::floor(row_half_angle * 2 / bristle_angle_delta);

        tread_row.tread_width = tread_width;
        tread_row.patch_half_length = row_half_length;

        auto patch_start_angle = spin_angleA.count * pi2 + contact_angle - row_half_angle;
        auto patch_end_angle = spin_angleA.count * pi2 + contact_angle + row_half_angle;

        // Keep angle in [0, 2π].
        /* if (patch_start_angle < 0) {
            patch_start_angle += 2 * pi;
        } else if (patch_start_angle > 2 * pi) {
            patch_start_angle -= 2 * pi;
        }

        if (patch_end_angle < 0) {
            patch_end_angle += 2 * pi;
        } else if (patch_end_angle > 2 * pi) {
            patch_end_angle -= 2 * pi;
        } */

        uint16_t start_idx;

        if (patch_start_angle < 0) {
            auto angle = pi2 + patch_start_angle + (spin_angleA.count - 1) * pi2;
            start_idx = uint16_t(std::ceil(angle / bristle_angle_delta));
        } else {
            auto angle = patch_start_angle + spin_angleA.count * pi2;
            start_idx = uint16_t(std::ceil(angle / bristle_angle_delta));
        }

        uint16_t end_idx = start_idx + row_num_bristles;

        std::cout << "start_idx: " << start_idx << " | end_idx:" << end_idx << std::endl;

        /* if (patch_end_angle < spin_angleA) {
            auto angle = pi2 - (spin_angleA - patch_end_angle) + (spin_angleA.count - 1) * pi2;
            end_idx = uint16_t(std::ceil(angle / bristle_angle_delta));
        } else {
            auto angle = patch_end_angle - spin_angleA + spin_angleA.count * pi2;
            end_idx = uint16_t(std::ceil(angle / bristle_angle_delta));
        } */

        // Remove existing bristles that are outside the contact patch.
        for (auto iter = tread_row.bristles.begin(); iter != tread_row.bristles.end();) {
            auto idx = iter->first;
            if (idx < start_idx || idx >= end_idx) {
                iter = tread_row.bristles.erase(iter);
            } else {
                ++iter;
            }
        }

        const auto row_area = scalar(2) * row_half_length * tread_width;
        const auto tread_area = row_num_bristles > 0 ? row_area / row_num_bristles : scalar(0);

        //auto spin_count_angle_delta = (spin_angleA.count - tread_row.prev_spin_count) * pi2;
        auto prev_patch_start_angle = tread_row.prev_spin_count * pi2 + tread_row.prev_contact_angle - tread_row.prev_range_half_angle;
        auto prev_patch_end_angle   = tread_row.prev_spin_count * pi2 + tread_row.prev_contact_angle + tread_row.prev_range_half_angle;

        std::cout << "range: " << patch_start_angle << ", " << patch_end_angle << std::endl;
        std::cout << "prev: " << prev_patch_start_angle << ", " << prev_patch_end_angle << std::endl;

        // Introduce new bristles into the contact patch.
        for (uint16_t j = start_idx; j != end_idx; ++j) {
            ++total_bristles;

            brush_bristle *bristle;

            if (tread_row.bristles.count(j)) {
                bristle = &tread_row.bristles[j];
            } else {
                const auto bristle_angle = scalar(j) * bristle_angle_delta;

                // Calculate time when bristle came into contact with ground,
                // i.e. the moment it entered the contact patch.
                scalar entry_dt = 0;
/* 
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
                } */

                auto bristle_pivot = vector3{row_x, std::sin(bristle_angle) * cyl.radius, std::cos(bristle_angle) * cyl.radius};
                auto rA = bristle_pivot;

                auto entry_pos = posA + linvelA * entry_dt;
                auto entry_orn = integrate(ornA, angvelA, entry_dt);
                auto entry_spin_angle = spin_angleA + spinA * entry_dt;
                auto entry_spin_orn = entry_orn * quaternion_axis_angle(vector3_x, entry_spin_angle);
                auto entry_bristle_pos = entry_pos + rotate(entry_spin_orn, bristle_pivot);
                entry_bristle_pos -= normal * dot(entry_bristle_pos - pB, normal);
                auto rB = rotate(conjugate(integrate(ornB, angvelB, entry_dt)), entry_bristle_pos - (posB + linvelB * entry_dt));

                tread_row.bristles[j] = brush_bristle {rA, rB};
                bristle = &tread_row.bristles[j];
            }

            auto bristle_root = posA + rotate(spin_ornA, bristle->pivotA);
            bristle_root -= normal * dot(bristle_root - pB, normal);
            auto pivotA = bristle_root - posA;

            auto bristle_tip = posB + rotate(ornB, bristle->pivotB);
            bristle_tip -= normal * dot(bristle_tip - pB, normal);
            auto pivotB = bristle_tip - posB;

            auto velA = linvelA + cross(spin_angvelA, pivotA);
            auto velB = linvelB + cross(angvelB, pivotB);
            auto relvel = velA - velB;
            auto tanrelvel = relvel - normal * dot(relvel, normal);
            auto tanrelspd = length(tanrelvel);
            bristle->friction = m_friction_coefficient / (1 + m_speed_sensitivity * tanrelspd);

            auto normal_pressure = row_half_length > 0 ? 
                                   (normal_force / num_tread_rows) / 
                                   (tread_width * 2 * row_half_length * (1 - 0.25/2 - 0.25/3)) : 
                                   scalar(0);
            auto max_friction_force = bristle->friction * normal_pressure * tread_area;
            auto spring_force = vector3_zero;

            auto bristle_defl = bristle_root - bristle_tip;
            auto bristle_defl_len2 = length2(bristle_defl);

            if (bristle_defl_len2 > EDYN_EPSILON) {
                spring_force = m_tread_stiffness * tread_area * bristle_defl;

                if (length2(spring_force) > max_friction_force * max_friction_force) {
                    auto error = std::sqrt(bristle_defl_len2);
                    auto dir = bristle_defl / error;
                    auto max_tread_defl = bristle->friction * normal_pressure / m_tread_stiffness;
                    bristle_defl = dir * max_tread_defl;
                    bristle_defl_len2 = length2(bristle_defl);
                    error = std::sqrt(bristle_defl_len2);
                    spring_force = m_tread_stiffness * tread_area * bristle_defl;
                    bristle_tip = bristle_root - bristle_defl;
                    pivotB = bristle_tip - posB;

                    bristle->pivotB = rotate(ornB_conj, pivotB);
                }
            }

            scalar tread_damping = 500000;
            bristle->damping_force = vector3_zero;//tanrelvel * tread_damping * tread_area;
            auto force = spring_force + bristle->damping_force;
            
            lon_force += dot(m_lon_dir, force);
            lat_force += dot(m_lat_dir, force);
            aligning_torque += dot(cross(bristle_root - m_patch_center, force), normal);

            lon_error += dot(m_lon_dir, bristle_defl);
            lat_error += dot(m_lat_dir, bristle_defl);
            aligning_error += dot(cross(bristle_root - m_patch_center, bristle_tip - m_patch_center), normal);

            bristle->root = bristle_root;
            bristle->deflection = bristle_defl;
            bristle->tread_area = tread_area;
        }

        tread_row.prev_contact_angle = contact_angle;
        tread_row.prev_range_half_angle = row_half_angle;
        tread_row.prev_spin_count = spin_angleA.count;
    }

    if (total_bristles > 0) {
        lon_error /= total_bristles;
        lat_error /= total_bristles;
        aligning_error /= total_bristles;
    }

    // Longitudinal.
    {
        auto p = cross(rA, m_lon_dir);
        auto q = cross(rB, m_lon_dir);
        auto impulse = std::abs(lon_force * dt);

        auto &row = registry.get<constraint_row>(con.row[1]);
        row.J = {m_lon_dir, p, -m_lon_dir, -q};
        row.error = lon_error / dt;
        row.lower_limit = -impulse;
        row.upper_limit = impulse;
    }

    // Lateral.
    {
        auto p = cross(rA, m_lat_dir);
        auto q = cross(rB, m_lat_dir);
        auto impulse = std::abs(lat_force * dt);

        auto &row = registry.get<constraint_row>(con.row[2]);
        row.J = {m_lat_dir, p, -m_lat_dir, -q};
        row.error = lat_error / dt;
        row.lower_limit = -impulse;
        row.upper_limit = impulse;
    }

    // Aligning moment.
    {
        // TODO: Also apply force on center of mass due to torque not necessarily
        // being aligned with the center of mass (e.g. when there's non-zero camber).
        auto impulse = std::abs(aligning_torque * dt);
        
        auto &row = registry.get<constraint_row>(con.row[3]);
        row.J = {vector3_zero, -normal, vector3_zero, normal};
        row.error = aligning_error / dt;
        row.lower_limit = -impulse;
        row.upper_limit = impulse;
    }
}

void contact_patch_constraint::iteration(entt::entity entity, constraint &con, 
                                         const relation &rel, entt::registry &registry, 
                                         scalar dt) {
    if (con.num_rows == 0) {
        return;
    }

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
            auto max_friction_force = bristle.friction * normal_pressure * bristle.tread_area;
            auto dl2 = length2(bristle.deflection);
            auto spring_force = vector3_zero;

            if (dl2 > EDYN_EPSILON) {
                spring_force = m_tread_stiffness * bristle.tread_area * bristle.deflection;

                if (length2(spring_force) > max_friction_force * max_friction_force) {
                    auto dir = bristle.deflection / std::sqrt(dl2);
                    auto max_tread_defl = bristle.friction * normal_pressure / m_tread_stiffness;
                    bristle.deflection = dir * max_tread_defl;
                    spring_force = m_tread_stiffness * bristle.tread_area * bristle.deflection;
                }
            }

            auto force = spring_force + bristle.damping_force;
            
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
    }
}

}