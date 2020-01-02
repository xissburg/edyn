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
    const auto &shapeA = registry.get<shape>(rel.entity[0]);
    const auto &cyl = std::get<cylinder_shape>(shapeA.var);
    
    const auto &linvelA = registry.get<linvel>(rel.entity[0]);
    const auto &angvelA = registry.get<angvel>(rel.entity[0]);

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
    const scalar perimeter = pi * cyl.radius * 2;
    const size_t num_bristles = std::floor(perimeter / desired_spacing);
    const scalar bristle_spacing = perimeter / num_bristles;
    auto center_bristle_origin = angle * cyl.radius;
    size_t center_idx = std::round(center_bristle_origin / bristle_spacing);

    auto &normal_row = registry.get<constraint_row>(normal_row_entity);
    auto normal_force = normal_row.impulse / dt;
    
    for (size_t i = 0; i < num_rows; ++i) {
        auto row_x = tread_width * (i + 0.5);
        auto row_center_cyl = p0 - axis * row_x;
        auto defl = dot(row_center_cyl - pB, -up);
        auto row_half_length = 0.4 * cyl.radius * (defl * r0_inv + 2.25 * std::sqrt(defl * r0_inv));
        auto row_center = row_center_cyl + up * defl;
        auto row_start = row_center + forward * row_half_length;
        auto tread_length = row_half_length * 2 / num_bristles;

        auto normal_pressure = normal_force / (contact_width * 2 * row_half_length * (1 - 0.25/2 - 0.25/3));

        auto patch_start = std::fmod(center_bristle_origin - row_half_length, perimeter);
        size_t start_idx = size_t(std::ceil(patch_start / bristle_spacing)) % num_bristles;

        auto patch_end = std::fmod(center_bristle_origin + row_half_length, perimeter);
        size_t end_idx = size_t(std::ceil(patch_end / bristle_spacing)) % num_bristles;

        for (size_t j = start_idx; j != end_idx; j = (j+1) % num_bristles) {
            auto pair = std::make_pair(i, j);
            brush_bristle *bristle;

            if (bristles.count(pair)) {
                bristle = &bristles[pair];
            } else {
                auto bristle_origin = j * bristle_spacing;
                scalar s;

                if (prev_patch_start[i] < bristle_origin) {
                    s = (bristle_origin - prev_patch_start[i]) / (patch_start - prev_patch_start[i]);
                } else { // if (prev_patch_end[i] > bristle_origin) {
                    s = (bristle_origin - prev_patch_end[i]) / (patch_end - prev_patch_end[i]);
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

                bristles[pair] = brush_bristle {ent, rA, rB};
                bristle = &bristles[pair];
            }

            auto bristle_root = posA + rotate(ornA, bristle->pivotA);
            auto bristle_tip = posB + rotate(ornB, bristle->pivotB);
            auto d = bristle_root - bristle_tip;
            d -= normal * dot(d, normal);
            auto dl2 = length2(d);

            auto tread_area = tread_length * tread_width;
            auto tread_normal_pressure = normal_pressure / tread_area;
            auto friction_force = friction_coefficient * tread_normal_pressure * tread_area;
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

            auto impulse = force * dt;

            auto &row = registry.get_or_assign<constraint_row>(bristle->entity);
            row.J = {d, cross(bristle->pivotA, d), -d, -cross(bristle->pivotB, d)};
            row.error = scalar(0.5) * dl2 / dt;
            row.lower_limit = -impulse;
            row.upper_limit = impulse;
        }

        prev_patch_start[i] = patch_start;
        prev_patch_end[i] = patch_end;
    }
    
}

void contact_patch_constraint::iteration(entt::entity entity, constraint &con, 
                                         const relation &rel, entt::registry &registry, 
                                         scalar dt) {

}

}