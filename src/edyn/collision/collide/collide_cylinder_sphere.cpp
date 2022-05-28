#include "edyn/collision/collide.hpp"
#include "edyn/math/geom.hpp"
#include "edyn/math/math.hpp"
#include "edyn/shapes/cylinder_shape.hpp"

namespace edyn {

void collide(const cylinder_shape &shA, const sphere_shape &shB,
             const collision_context &ctx, collision_result &result) {
    const auto &posA = ctx.posA;
    const auto &ornA = ctx.ornA;
    const auto &posB = ctx.posB;
    const auto &ornB = ctx.ornB;
    const auto threshold = ctx.threshold;

    const auto cyl_axis = coordinate_axis_vector(shA.axis, ornA);
    const auto cyl_vertices = std::array<vector3, 2>{
        posA + cyl_axis * shA.half_length,
        posA - cyl_axis * shA.half_length
    };

    const auto v = cyl_vertices[1] - cyl_vertices[0]; // Direction vector of cylinder segment.
    const auto w = posB - cyl_vertices[0]; // Vector from initial point of cylinder segment to point sphere center.
    const auto denom = dot(v, v);
    EDYN_ASSERT(denom > EDYN_EPSILON);
    const auto t = dot(w, v) / denom;

    if (t > 0 && t < 1) {
        const auto p_cyl = cyl_vertices[0] + v * t;
        const auto dir = p_cyl - posB;
        const auto dist_sqr = length_sqr(dir);
        const auto min_dist = shA.radius + shB.radius + threshold;

        if (dist_sqr > min_dist * min_dist) {
            return;
        }

        const auto dist = std::sqrt(dist_sqr);
        const auto normal = dist_sqr > EDYN_EPSILON ? dir / dist : vector3_y;

        collision_result::collision_point point;
        point.pivotA = rotate(conjugate(ornA), p_cyl - normal * shA.radius - posA);
        point.pivotB = rotate(conjugate(ornB), normal * shB.radius);
        point.distance = dist - shA.radius - shB.radius;
        point.normal = normal;
        point.normal_attachment = contact_normal_attachment::none;
        point.featureA = {cylinder_feature::side_edge};
        result.add_point(point);
        return;
    }

    size_t cyl_face_idx = t < 0.5 ? 0 : 1;
    const auto disc_pos = cyl_vertices[cyl_face_idx];
    vector3 closest;
    const auto dist_sqr = closest_point_disc(disc_pos, ornA, shA.radius, shA.axis, posB, closest);
    const auto min_dist = shB.radius + threshold;

    if (dist_sqr > min_dist * min_dist) {
        return;
    }

    auto normal = closest - posB;
    const auto n_len_sqr = length_sqr(normal);
    const auto n_len = std::sqrt(n_len_sqr);
    normal = n_len_sqr > EDYN_EPSILON ? normal / n_len : cyl_axis * to_sign(t > 0.5);

    collision_result::collision_point point;
    point.pivotA = rotate(conjugate(ornA), closest - posA);
    point.pivotB = rotate(conjugate(ornB), normal * shB.radius);
    point.distance = n_len - shB.radius;
    point.normal = normal;

    // If the closest feature is the cylinder face, attach normal to A.
    // I.e. if the projection of the sphere center is inside the cap face.
    auto sphere_proj = project_plane(posB, posA, cyl_axis);

    if (distance_sqr(sphere_proj, posA) < shA.radius * shA.radius) {
        point.normal_attachment = contact_normal_attachment::normal_on_A;
        point.featureA = {cylinder_feature::face, cyl_face_idx};
    } else {
        point.normal_attachment = contact_normal_attachment::none;
        point.featureA = {cylinder_feature::side_edge, cyl_face_idx};
    }

    result.add_point(point);
}

void collide(const sphere_shape &shA, const cylinder_shape &shB,
             const collision_context &ctx, collision_result &result) {
    swap_collide(shA, shB, ctx, result);
}

}
