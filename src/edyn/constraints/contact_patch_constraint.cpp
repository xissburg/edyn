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

void contact_patch_constraint::prepare(entt::entity entity, constraint &con, 
                                       const relation &rel, entt::registry &registry,
                                       scalar dt) {
    auto &manifold = registry.get<contact_manifold>(entity);
    
    if (manifold.num_points == 0) {
        registry.destroy(normal_row_entity);

        for (size_t i = 0; i < con.num_rows; ++i) {
            registry.destroy(con.row[i]);
        }
        con.num_rows = 0;
        return;
    }

    const auto &posA = registry.get<position>(rel.entity[0]);
    const auto &ornA = registry.get<orientation>(rel.entity[0]);
    const auto &posB = registry.get<position>(rel.entity[1]);
    const auto &ornB = registry.get<orientation>(rel.entity[1]);
    const auto &normal = manifold.point[0].normalB;

    // Create non-penetration constraint.


    const auto &shapeA = registry.get<shape>(rel.entity[0]);
    const auto &cyl = std::get<cylinder_shape>(shapeA.var);
    
    const auto &linvelA = registry.get<linvel>(rel.entity[0]);
    const auto &angvelA = registry.get<angvel>(rel.entity[0]);
    const auto &linvelB = registry.get<linvel>(rel.entity[1]);
    const auto &angvelB = registry.get<angvel>(rel.entity[1]);

    const auto axis = rotate(ornA, vector3_x);
    const auto axis_hl = axis * cyl.half_length;
    auto p0 = support_point_circle(posA + axis_hl, ornA, cyl.radius, -normal);
    auto p1 = p0 - axis_hl * 2; // because circles are parallel
    auto pB = posB + rotate(ornB, manifold.point[0].pivotB);

    auto ornA_conj = conjugate(ornA);
    auto p0_obj = rotate(ornA_conj, p0 - posA);
    auto angle = std::atan2(p0_obj.z, p0_obj.y);

    auto proj0 = dot(p0 - pB, normal);
    auto proj1 = dot(p1 - pB, normal);

    // One of them's gotta be under the plane.
    EDYN_ASSERT(proj0 < 0 || proj1 < 0);
    
    // Intersect segment between `p0` and `p1` with contact plane to find
    // the extreme points along the width of the contact patch.
    if (proj0 > 0) {
        // `p0` is above the plane.
        p0 = p1 + axis_hl * proj0 / dot(axis_hl, normal);
    } else if (proj1 > 0) {
        // `p1` is above the plane.
        p1 = p0 + axis_hl * proj1 / dot(axis_hl, normal);
    }

    auto forward = normalize(cross(axis, normal));
    auto up = cross(forward, axis);

    auto contact_width = length(p0 - p1);
    auto tread_width = contact_width / num_rows;
    auto r0_inv = scalar(1) / cyl.radius;
    auto basis = matrix3x3_columns(axis, up, forward);
    auto orn = to_quaternion(basis);
    auto orn_conj = conjugate(orn);

    const auto ornB_conj = conjugate(ornB);
        
    const scalar desired_spacing = 0.01;
    const scalar half_perimeter = pi * cyl.radius;
    const scalar perimeter = half_perimeter * 2;
    const size_t num_bristles = std::floor(perimeter / desired_spacing);
    const scalar bristle_spacing = perimeter / num_bristles;
    auto center_bristle_origin = angle * cyl.radius;
    size_t center_idx = std::round(center_bristle_origin / bristle_spacing);

    auto &normal_row = registry.get<constraint_row>(normal_row_entity);
    auto normal_force = normal_row.impulse / dt;
    
    for (size_t i = 0; i < num_rows; ++i) {
        auto &tread_row = tread_rows[i];
        auto row_x = tread_width * (i + 0.5);
        auto row_center_cyl = p0 - axis * row_x;
        auto defl = dot(row_center_cyl - pB, -up);
        auto row_half_length = 0.4 * cyl.radius * (defl * r0_inv + 2.25 * std::sqrt(defl * r0_inv));
        auto row_center = row_center_cyl + up * defl;
        auto row_start = row_center + forward * row_half_length;
        auto tread_length = row_half_length * 2 / num_bristles;

        auto normal_pressure = normal_force / (tread_width * 2 * row_half_length * (1 - 0.25/2 - 0.25/3));

        auto row_half_arc = 2 * cyl.radius * std::acos(row_half_length / cyl.radius);
        auto patch_start = center_bristle_origin - row_half_arc;
        auto patch_end = center_bristle_origin + row_half_arc;

        auto wraps_around = patch_start < -half_perimeter || patch_end > half_perimeter;

        patch_start = std::fmod(patch_start, half_perimeter);
        patch_end = std::fmod(patch_end, half_perimeter);

        size_t start_idx = size_t(std::ceil(patch_start / bristle_spacing)) % num_bristles;
        size_t end_idx = size_t(std::ceil(patch_end / bristle_spacing)) % num_bristles;

        tread_row.tread_width = tread_width;
        tread_row.patch_half_length = row_half_length;

        // Remove existing bristles that are outside the contact patch.
        for (auto iter = tread_row.bristles.begin(); iter != tread_row.bristles.end();) {
            if (iter->first == i &&
               ((wraps_around &&  iter->first < start_idx && iter->first > end_idx) ||
               (!wraps_around && (iter->first < start_idx || iter->first > end_idx)))) {
                iter = tread_row.bristles.erase(iter);
            } else {
                ++iter;
            }
        }

        // Introduce new bristles into the contact patch.
        for (size_t j = start_idx; j != end_idx; j = (j+1) % num_bristles) {
            brush_bristle *bristle;

            if (tread_row.bristles.count(j)) {
                bristle = &tread_row.bristles[j];
            } else {
                auto bristle_origin = j * bristle_spacing;
                scalar s;

                if (tread_row.prev_patch_start < bristle_origin) {
                    s = (bristle_origin - tread_row.prev_patch_start) / (patch_start - tread_row.prev_patch_start);
                } else { // if (prev_patch_end[i] > bristle_origin) {
                    s = (bristle_origin - tread_row.prev_patch_end) / (patch_end - tread_row.prev_patch_end);
                }

                auto entry_dt = -s * dt;
                auto entry_pos = posA + linvelA * entry_dt;
                auto entry_orn = integrate(ornA, angvelA, entry_dt);

                auto bristle_angle = bristle_origin * r0_inv;
                auto bristle_pivot = vector3{row_x, std::cos(bristle_angle) * cyl.radius, std::sin(bristle_angle) * cyl.radius};
                
                auto entry_bristle_pos = entry_pos + rotate(entry_orn, bristle_pivot);
                auto rA = bristle_pivot;
                auto rB = rotate(ornB_conj, posB - entry_bristle_pos);

                auto ent = registry.create();
                con.row[con.num_rows++] = ent;

                tread_row.bristles[j] = brush_bristle {ent, rA, rB};
                bristle = &tread_row.bristles[j];
            }

            auto bristle_root = posA + rotate(ornA, bristle->pivotA);
            auto bristle_tip = posB + rotate(ornB, bristle->pivotB);
            auto d = bristle_root - bristle_tip;
            d -= normal * dot(d, normal); // project on contact plane
            auto dl2 = length2(d);
            
            auto velA = linvelA + cross(angvelA, bristle->pivotA);
            auto velB = linvelB + cross(angvelB, bristle->pivotB);
            auto relvel = velA - velB;
            auto tanrelvel = relvel - normal * dot(relvel, normal);
            auto tanrelspd = length(tanrelvel);
            bristle->friction = friction_coefficient / (1 + speed_sensitivity * tanrelspd);

            auto tread_area = tread_length * tread_width;
            auto tread_normal_pressure = normal_pressure / tread_area;
            auto friction_force = bristle->friction * tread_normal_pressure * tread_area;
            auto error = std::sqrt(dl2);
            auto force = std::abs(tread_stiffness * tread_area * error);

            if (force > friction_force) {
                force = friction_force;
                auto dir = d / error;
                bristle_tip = dir * friction_force / (tread_stiffness * tread_area);

                bristle->pivotB = rotate(ornB_conj, posB - bristle_tip);
                d = bristle_root - bristle_tip;
                d -= normal * dot(d, normal);
                dl2 = length2(d);

                error = std::sqrt(dl2);
            }

            bristle->force = force;
            bristle->tread_area = tread_area;

            auto impulse = force * dt;

            auto &row = registry.get_or_assign<constraint_row>(bristle->entity);
            row.J = {d, cross(bristle->pivotA, d), -d, -cross(bristle->pivotB, d)};
            row.error = scalar(0.5) * dl2 / dt;
            row.lower_limit = -impulse;
            row.upper_limit = impulse;
        }

        tread_row.prev_patch_start = patch_start;
        tread_row.prev_patch_end = patch_end;
    }
    
}

void contact_patch_constraint::iteration(entt::entity entity, constraint &con, 
                                         const relation &rel, entt::registry &registry, 
                                         scalar dt) {
    auto &normal_row = registry.get<constraint_row>(normal_row_entity);
    auto normal_force = normal_row.impulse / dt;

    for (auto &tread_row : tread_rows) {
        for (auto &kv : tread_row.bristles) {
            auto tread_idx = kv.first;
            auto &bristle = kv.second;

            auto normal_pressure = normal_force / (tread_row.tread_width * 2 * tread_row.patch_half_length * scalar(1 - 0.25/2 - 0.25/3));
            auto tread_normal_pressure = normal_pressure / bristle.tread_area;
            auto friction_force = bristle.friction * tread_normal_pressure * bristle.tread_area;
            auto force = std::min(friction_force, bristle.force);
            auto impulse = force * dt;

            auto &row = registry.get_or_assign<constraint_row>(bristle.entity);
            row.lower_limit = -impulse;
            row.upper_limit = impulse;
        }
    }
}

}