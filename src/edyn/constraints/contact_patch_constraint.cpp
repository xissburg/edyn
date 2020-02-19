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
    if (normal_row_entity != entt::null) {
        registry.destroy(normal_row_entity);
        normal_row_entity = entt::null;
    }

    for (size_t i = 1; i < con.num_rows; ++i) {
        registry.destroy(con.row[i]);
    }
    con.num_rows = 0;

    for (size_t i = 0; i < num_tread_rows; ++i) {
        auto &tread_row = tread_rows[i];
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

    const auto &posA = registry.get<position>(rel.entity[0]);
    const auto &ornA = registry.get<orientation>(rel.entity[0]);
    
    const auto axis = rotate(ornA, vector3_x);
    
    const auto &spin_angleA = registry.get<spin_angle>(rel.entity[0]);
    const auto spin_ornA = ornA * quaternion_axis_angle(vector3_x, spin_angleA);

    const auto &posB = registry.get<position>(rel.entity[1]);
    const auto &ornB = registry.get<orientation>(rel.entity[1]);
    const auto ornB_conj = conjugate(ornB);

    const auto normal = rotate(ornB, manifold.point[0].normalB);

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
    auto pB = posB + rotate(ornB, manifold.point[0].pivotB);

    // Support point in object space.
    const auto spin_ornA_conj = conjugate(spin_ornA);
    auto p0_obj = rotate(spin_ornA_conj, p0 - posA);

    // Support point angle in circle space where z points forward and y is up.
    scalar angle = std::atan2(p0_obj.y, p0_obj.z);

    // Transform angle from [-π, π] to [0, 2π].
    if (angle < 0) {
        angle += 2 * pi;
    }

    auto proj0 = dot(p0 - pB, normal);
    auto proj1 = dot(p1 - pB, normal);

    // One of them's gotta be under the plane.
    /* if (proj0 > 0 && proj1 > 0) {
        clear(registry, con);
        return;
    } */

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

    // Create non-penetration constraint.
    if (normal_row_entity == entt::null) {
        normal_row_entity = registry.create();
        auto &normal_row = registry.assign<constraint_row>(normal_row_entity);
        normal_row.entity = rel.entity;
        normal_row.priority = 0;
        con.row[0] = normal_row_entity;
        con.num_rows = 1;
    }

    // Setup non-penetration constraint.
    auto patch_center = (p0 + p1) * 0.5;
    auto rA = patch_center - posA;

    auto patch_center_proj = patch_center - normal * dot(patch_center - pB, normal);
    auto rB = patch_center_proj - posB;
    
    auto &normal_row = registry.get<constraint_row>(normal_row_entity);
    normal_row.J = {normal, cross(rA, normal), -normal, -cross(rB, normal)};
    normal_row.lower_limit = 0;

    auto penetration = dot(posA + rA - posB - rB, normal);
    auto spring_force = penetration * stiffness;
    auto normal_relvel = dot(linvelA + cross(angvelA, rA) - linvelB - cross(angvelB, rB), normal);
    auto damper_force = normal_relvel * damping;
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

    auto contact_width = length(p0 - p1);
    auto tread_width = contact_width / num_tread_rows;
    auto r0_inv = scalar(1) / cyl.radius;

    const scalar half_perimeter = pi * cyl.radius;
    const scalar perimeter = half_perimeter * 2;
    const scalar desired_spacing = 0.01;
    const size_t num_bristles = std::floor(perimeter / desired_spacing);
    const scalar bristle_spacing = perimeter / num_bristles;
    const scalar bristle_angle = bristle_spacing * r0_inv;
    const auto tread_area = bristle_spacing * tread_width;

    // Position of deepest point along the perimeter of the circle.
    auto center_bristle_origin = angle * cyl.radius;

    // Index of bristle closest to deepest/central point.
    size_t center_idx = std::round(center_bristle_origin / bristle_spacing);

    auto normal_force = normal_row.impulse / dt;
    
    // Update bristles for each row.
    for (size_t i = 0; i < num_tread_rows; ++i) {
        auto &tread_row = tread_rows[i];
        auto row_x = row_start + tread_width * (i + 0.5);
        auto row_center_cyl = p0 - axis_hl + axis * row_x;
        auto defl = std::clamp(dot(row_center_cyl - pB, -up), scalar(0), cyl.radius / 2);
        auto row_half_length = scalar(0.4) * cyl.radius * (defl * r0_inv + scalar(2.25) * std::sqrt(defl * r0_inv));

        auto row_half_angle = std::asin(row_half_length / cyl.radius);
        auto patch_start_angle = angle - row_half_angle;
        auto patch_end_angle = angle + row_half_angle;

        // Keep angle in [0, 2π].
        if (patch_start_angle < 0) {
            patch_start_angle += 2 * pi;
        } else if (patch_start_angle > 2 * pi) {
            patch_start_angle -= 2 * pi;
        }

        if (patch_end_angle < 0) {
            patch_end_angle += 2 * pi;
        } else if (patch_end_angle > 2 * pi) {
            patch_end_angle -= 2 * pi;
        }

        tread_row.tread_width = tread_width;
        tread_row.patch_half_length = row_half_length;

        const bool wraps_around = patch_start_angle > patch_end_angle;

        // Remove existing bristles that are outside the contact patch.
        for (auto iter = tread_row.bristles.begin(); iter != tread_row.bristles.end();) {
            auto ang = scalar(iter->first) * bristle_angle;

            if ((!wraps_around && (patch_start_angle < ang && ang < patch_end_angle)) ||
                ( wraps_around && (patch_start_angle < ang || ang < patch_end_angle))) {
                ++iter;
            } else {
                auto row_end_iter = con.row.begin() + con.num_rows;
                auto row_iter = std::find(con.row.begin(), row_end_iter, iter->second.entity);
                EDYN_ASSERT(row_iter != row_end_iter);
                EDYN_ASSERT(con.num_rows > 0);
                --con.num_rows;
                *row_iter = con.row[con.num_rows]; // swap with last
                registry.destroy(iter->second.entity);
                iter = tread_row.bristles.erase(iter);
            }
        }

        size_t start_idx = size_t(std::ceil(patch_start_angle / bristle_angle)) % num_bristles;
        size_t end_idx = size_t(std::ceil(patch_end_angle / bristle_angle)) % num_bristles;

        // Introduce new bristles into the contact patch.
        for (size_t j = start_idx; j != end_idx; j = (j+1) % num_bristles) {
            brush_bristle *bristle;

            if (tread_row.bristles.count(j)) {
                bristle = &tread_row.bristles[j];
            } else {
                auto ang = scalar(j) * bristle_angle;

                // It must've been outside the patch in the previous step.
                //EDYN_ASSERT((!wraps_around && (tread_row.prev_patch_start_angle < ang && ang < tread_row.prev_patch_end_angle)) ||
                //            ( wraps_around && (tread_row.prev_patch_start_angle < ang || ang < tread_row.prev_patch_end_angle)));

                // Calculate time when bristle came into contact with ground,
                // i.e. the moment it entered the contact patch.
                scalar entry_dt = 0;

                auto dist_start = std::min(std::abs(tread_row.prev_patch_start_angle - ang), 
                                           std::abs(tread_row.prev_patch_start_angle - (ang + pi2)));
                auto dist_end = std::min(std::abs(tread_row.prev_patch_end_angle - ang), 
                                         std::abs(tread_row.prev_patch_end_angle - (ang + pi2)));

                if (dist_start < dist_end) { // entered through the front
                    auto dist_patch_start = std::min(std::abs(tread_row.prev_patch_start_angle - patch_start_angle), 
                                                     std::abs(tread_row.prev_patch_start_angle - (patch_start_angle + pi2)));
                    if (dist_patch_start > EDYN_EPSILON) {
                        entry_dt = -dt * (1 - dist_start / dist_patch_start);
                    }
                } else { // entered through the back
                    auto dist_patch_end = std::min(std::abs(tread_row.prev_patch_end_angle - patch_end_angle), 
                                                   std::abs(tread_row.prev_patch_end_angle - (patch_end_angle + pi2)));
                    if (dist_patch_end > EDYN_EPSILON) {
                        entry_dt = -dt * (1 - dist_end / dist_patch_end);
                    }
                }

                auto bristle_pivot = vector3{row_x, std::sin(ang) * cyl.radius, std::cos(ang) * cyl.radius};
                auto rA = bristle_pivot;

                auto entry_pos = posA + linvelA * entry_dt;
                auto entry_orn = integrate(ornA, angvelA, entry_dt);
                auto entry_spin_angle = spin_angleA + spinA * entry_dt;
                auto entry_spin_orn = entry_orn * quaternion_axis_angle(vector3_x, entry_spin_angle);
                auto entry_bristle_pos = entry_pos + rotate(entry_spin_orn, bristle_pivot);
                entry_bristle_pos -= normal * dot(entry_bristle_pos - pB, normal);
                auto rB = rotate(conjugate(integrate(ornB, angvelB, entry_dt)), entry_bristle_pos - (posB + linvelB * entry_dt));

                auto ent = registry.create();
                con.row[con.num_rows++] = ent;

                tread_row.bristles[j] = brush_bristle {ent, rA, rB};
                bristle = &tread_row.bristles[j];
            }

            auto bristle_root = posA + rotate(spin_ornA, bristle->pivotA);
            bristle_root -= normal * dot(bristle_root - pB, normal);
            auto pivotA = bristle_root - posA;

            auto bristle_tip = posB + rotate(ornB, bristle->pivotB);
            bristle_tip -= normal * dot(bristle_tip - pB, normal);
            auto pivotB = bristle_tip - posB;

            auto d = bristle_root - bristle_tip;
            auto dl2 = length2(d);

            auto velA = linvelA + cross(spin_angvelA, pivotA);
            auto velB = linvelB + cross(angvelB, pivotB);
            auto relvel = velA - velB;
            auto tanrelvel = relvel - normal * dot(relvel, normal);
            auto tanrelspd = length(tanrelvel);
            bristle->friction = friction_coefficient / (1 + speed_sensitivity * tanrelspd);

            auto row_area = scalar(2) * row_half_length * tread_width;
            auto normal_pressure = row_half_length > 0 ? (normal_force / num_tread_rows)  / (tread_width * 2 * row_half_length * (1 - 0.25/2 - 0.25/3)) : scalar(0);
            auto friction_force = bristle->friction * normal_pressure * tread_area;
            auto spring_force = vector3_zero;

            if (dl2 > EDYN_EPSILON) {
                spring_force = tread_stiffness * tread_area * d;

                if (length2(spring_force) > friction_force * friction_force) {
                    auto error = std::sqrt(dl2);
                    auto dir = d / error;
                    auto max_tread_defl = bristle->friction * normal_pressure / tread_stiffness;
                    d = dir * max_tread_defl;
                    dl2 = length2(d);
                    error = std::sqrt(dl2);
                    spring_force = tread_stiffness * tread_area * d;
                    bristle_tip = bristle_root - d;
                    pivotB = bristle_tip - posB;

                    bristle->pivotB = rotate(ornB_conj, pivotB);
                }
            }

            if (dl2 <= EDYN_EPSILON) {
                d = vector3_x;
            }

            scalar tread_damping = 500000;
            auto damping_force = tanrelvel * tread_damping * tread_area;
            auto force = length(spring_force + damping_force);
            bristle->force = force;
            bristle->tread_area = tread_area;

            auto impulse = force * dt;

            auto &row = registry.get_or_assign<constraint_row>(bristle->entity);
            row.priority = 1;
            row.entity = rel.entity;
            row.use_spin[0] = true;
            row.use_spin[1] = true;
            row.J = {d, cross(pivotA, d), -d, -cross(pivotB, d)};
            row.error = scalar(0.5) * dl2 / dt;
            row.lower_limit = -impulse;
            row.upper_limit = impulse;
        }

        tread_row.prev_patch_start_angle = patch_start_angle;
        tread_row.prev_patch_end_angle = patch_end_angle;
    }
}

void contact_patch_constraint::iteration(entt::entity entity, constraint &con, 
                                         const relation &rel, entt::registry &registry, 
                                         scalar dt) {
    if (normal_row_entity == entt::null) {
        return;
    }

    auto &normal_row = registry.get<constraint_row>(normal_row_entity);
    auto normal_force = normal_row.impulse / dt;

    for (auto &tread_row : tread_rows) {
        for (auto &kv : tread_row.bristles) {
            auto tread_idx = kv.first;
            auto &bristle = kv.second;

            auto normal_pressure = tread_row.patch_half_length > 0 ? 
                                  (normal_force / num_tread_rows)  / (tread_row.tread_width * tread_row.patch_half_length * scalar(2 * (1 - 0.25/2 - 0.25/3))) : 
                                  scalar(0);
            auto friction_force = bristle.friction * normal_pressure * bristle.tread_area;
            auto force = std::min(friction_force, bristle.force);
            auto impulse = force * dt;

            auto &row = registry.get_or_assign<constraint_row>(bristle.entity);
            row.lower_limit = -impulse;
            row.upper_limit = impulse;
        }
    }
}

}