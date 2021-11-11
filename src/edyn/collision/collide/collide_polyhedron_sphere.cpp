#include "edyn/collision/collide.hpp"
#include "edyn/math/geom.hpp"
#include "edyn/math/constants.hpp"
#include "edyn/math/transform.hpp"
#include "edyn/util/shape_util.hpp"

namespace edyn {

void collide(const polyhedron_shape &shA, const sphere_shape &shB,
             const collision_context &ctx, collision_result &result) {
    // Convex polyhedron vs sphere. A direction of greatest projection distance
    // is found among all face normals and a closest point is found in the face
    // in that direction looking through its Voronoi regions.
    // All calculations done in the polyhedron's space.
    const auto posB = to_object_space(ctx.posB, ctx.posA, ctx.ornA);
    const auto ornB = conjugate(ctx.ornA) * ctx.ornB;
    auto threshold = ctx.threshold;
    auto &meshA = *shA.mesh;

    scalar distance = -EDYN_SCALAR_MAX;
    scalar projection_poly = EDYN_SCALAR_MAX;
    auto sep_axis = vector3_zero;

    // Face normals of polyhedron.
    for (size_t i = 0; i < meshA.relevant_normals.size(); ++i) {
        auto normalA = -meshA.relevant_normals[i]; // Point towards polyhedron.
        auto &vertexA = meshA.vertices[meshA.relevant_indices[i]];

        auto projA = dot(vertexA, normalA);
        auto projB = dot(posB, normalA) + shB.radius;
        auto dist = projA - projB;

        if (dist > distance) {
            distance = dist;
            projection_poly = projA;
            sep_axis = normalA;
        }
    }

    if (distance > threshold) {
        return;
    }

    auto polygon = point_cloud_support_polygon(
        meshA.vertices.begin(), meshA.vertices.end(), vector3_zero,
        sep_axis, projection_poly, true, support_feature_tolerance);

    EDYN_ASSERT(polygon.hull.size() > 2);

    auto posB_plane = to_vector2_xz(to_object_space(posB, polygon.origin, polygon.basis));
    vector2 closest;
    bool inside_face = !closest_point_polygon(polygon, posB_plane, closest);

    if (inside_face) {
        auto pivotA = project_plane(posB, polygon.origin, sep_axis);
        auto normalB = rotate(conjugate(ornB), sep_axis);
        auto pivotB = normalB * shB.radius;
        auto normal = rotate(ctx.ornA, sep_axis);
        result.add_point({pivotA, pivotB, normal, distance, contact_normal_attachment::normal_on_A});
        return;
    }

    // Sphere is closer to an edge or vertex. Calculate new separating axis
    // and recalculate distance.
    auto pivotA = to_world_space(to_vector3_xz(closest),
                                 polygon.origin, polygon.basis);
    auto new_sep_axis = pivotA - posB;
    auto new_sep_axis_len_sqr = length_sqr(new_sep_axis);

    if (new_sep_axis_len_sqr > EDYN_EPSILON) {
        auto new_sep_axis_len = std::sqrt(new_sep_axis_len_sqr);
        new_sep_axis /= new_sep_axis_len;
        distance = new_sep_axis_len - shB.radius;

        if (distance > threshold) {
            return;
        }
    } else {
        new_sep_axis = sep_axis;
        pivotA = project_plane(posB, polygon.origin, new_sep_axis);
    }

    auto normalB = rotate(conjugate(ornB), new_sep_axis);
    auto pivotB = normalB * shB.radius;
    auto normal = rotate(ctx.ornA, new_sep_axis);

    result.add_point({pivotA, pivotB, normal, distance, contact_normal_attachment::none});
}

void collide(const sphere_shape &shA, const polyhedron_shape &shB,
             const collision_context &ctx, collision_result &result) {
    swap_collide(shA, shB, ctx, result);
}

}
