#include "edyn/collision/collide.hpp"
#include "edyn/math/geom.hpp"
#include "edyn/math/quaternion.hpp"
#include "edyn/math/vector3.hpp"
#include "edyn/util/shape_util.hpp"
#include "edyn/math/math.hpp"
#include "edyn/math/constants.hpp"

namespace edyn {

collision_result collide(const polyhedron_shape &shA, const sphere_shape &shB,
                         const collision_context &ctx) {
    const auto posA = vector3_zero;
    const auto &ornA = ctx.ornA;
    const auto posB = ctx.posB - ctx.posA;
    const auto &ornB = ctx.ornB;
    auto threshold = ctx.threshold;
    auto &rmeshA = ctx.rmeshA->get();

    scalar distance = -EDYN_SCALAR_MAX;
    scalar projection_poly = EDYN_SCALAR_MAX;
    auto sep_axis = vector3_zero;

    // Face normals of polyhedron.
    for (size_t i = 0; i < shA.mesh->num_faces(); ++i) {
        auto normal_world = -rmeshA.normals[i]; // Point towards polyhedron.
        auto vertex_idx = shA.mesh->first_vertex_index(i);
        auto &vertex_world = rmeshA.vertices[vertex_idx];

        auto projA = dot(vertex_world, normal_world);
        auto projB = dot(posB, normal_world) + shB.radius;
        auto dist = projA - projB;

        if (dist > distance) {
            distance = dist;
            projection_poly = projA;
            sep_axis = normal_world;
        }
    }

    if (distance > threshold) {
        return {};
    }

    auto result = collision_result{};

    auto polygon = point_cloud_support_polygon<true>(
        rmeshA.vertices.begin(), rmeshA.vertices.end(), vector3_zero,
        sep_axis, projection_poly, true, support_polygon_tolerance);

    EDYN_ASSERT(polygon.hull.size() > 2);

    auto posB_plane = to_vector2_xz(to_object_space(posB, polygon.origin, polygon.basis));
    vector2 closest;
    bool inside_face = true;

    for (auto i = 0; i < polygon.hull.size(); ++i) {
        auto j = (i + 1) % polygon.hull.size();
        auto i0 = polygon.hull[i];
        auto i1 = polygon.hull[j];
        auto &v0 = polygon.plane_vertices[i0];
        auto &v1 = polygon.plane_vertices[i1];

        auto e0 = v1 - v0;
        // Vertices are oriented counter-clockwise. Rotate edge clockwise
        // to obtain a vector that points outside the convex polygon.
        auto n0 = -orthogonal(e0);

        if (dot(posB_plane - v0, n0) < 0) {
            continue;
        }

        if (dot(posB_plane - v0, e0) > 0) {
            if (dot(posB_plane - v1, e0) < 0) {
                auto t = dot(posB_plane - v0, e0) / dot(e0, e0);
                closest = lerp(v0, v1, t);
                inside_face = false;
                break;
            } else {
                auto k = (i + 2) % polygon.hull.size();
                auto i2 = polygon.hull[k];
                auto &v2 = polygon.plane_vertices[i2];
                auto e1 = v2 - v1;

                if (dot(posB_plane - v1, e1) < 0) {
                    closest = v1;
                    inside_face = false;
                    break;
                }
            }
        }
    }

    if (inside_face) {
        auto pivotB_world = posB + sep_axis * shB.radius;
        auto pivotB = to_object_space(pivotB_world, posB, ornB);
        auto pivotA_world = project_plane(pivotB_world, polygon.origin, sep_axis);
        auto pivotA = to_object_space(pivotA_world, posA, ornA);
        auto normalB = rotate(conjugate(ornB), sep_axis);
        result.add_point({pivotA, pivotB, normalB, distance});
    } else {
        auto closest3 = vector3{closest.x, 0, closest.y};
        auto closest_world = to_world_space(closest3, polygon.origin, polygon.basis);
        sep_axis = closest_world - posB;
        distance = length(sep_axis);
        sep_axis /= distance;
        distance -= shB.radius;
        auto normalB = rotate(conjugate(ornB), sep_axis);
        auto pivotA = to_object_space(closest_world, posA, ornA);
        auto pivotB = normalB * shB.radius;
        result.add_point({pivotA, pivotB, normalB, distance});
    }

    return result;
}

collision_result collide(const sphere_shape &shA, const polyhedron_shape &shB, 
                         const collision_context &ctx) {
    return swap_collide(shA, shB, ctx);
}

}
