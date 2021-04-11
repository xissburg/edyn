#include "edyn/collision/collide.hpp"
#include "edyn/math/quaternion.hpp"
#include "edyn/util/shape_util.hpp"
#include "edyn/math/geom.hpp"

namespace edyn {

collision_result collide(const polyhedron_shape &shA, const capsule_shape &shB,
                         const collision_context &ctx) {
    const auto posA = vector3_zero;
    const auto &ornA = ctx.ornA;
    const auto posB = ctx.posB - ctx.posA;
    const auto &ornB = ctx.ornB;
    auto threshold = ctx.threshold;
    auto &rmeshA = ctx.rmeshA->get();

    auto capsule_axis = quaternion_x(ornB);
    vector3 capsule_vertices[] = {
        posB - capsule_axis * shB.half_length,
        posB + capsule_axis * shB.half_length
    };

    scalar distance = -EDYN_SCALAR_MAX;
    scalar projection_poly = EDYN_SCALAR_MAX;
    auto sep_axis = vector3_zero;

    // Face normals of polyhedron.
    for (size_t i = 0; i < shA.mesh->num_faces(); ++i) {
        auto normal_world = -rmeshA.normals[i]; // Point towards polyhedron.
        auto vertex_idx = shA.mesh->first_vertex_index(i);
        auto &vertex_world = rmeshA.vertices[vertex_idx];

        auto projA = dot(vertex_world, normal_world);
        auto projB = std::max(dot(capsule_vertices[0], normal_world),
                              dot(capsule_vertices[1], normal_world)) + shB.radius;
        auto dist = projA - projB;

        if (dist > distance) {
            distance = dist;
            projection_poly = projA;
            sep_axis = normal_world;
        }
    }

    // Edges vs capsule axis
    for (size_t i = 0; i < shA.mesh->num_edges(); ++i) {
        auto [vertexA0, vertexA1] = shA.mesh->get_edge(rmeshA, i);
        auto edgeA = vertexA1 - vertexA0;
        auto dir = cross(edgeA, capsule_axis);
        auto dir_len_sqr = length_sqr(dir);

        if (!(dir_len_sqr > EDYN_EPSILON)) {
            continue;
        }

        dir /= std::sqrt(dir_len_sqr);

        if (dot(posA - posB, dir) < 0) {
            // Make it point towards A.
            dir *= -1;
        }

        auto projA = -point_cloud_support_projection(rmeshA.vertices, -dir);
        auto projB = std::max(dot(capsule_vertices[0], dir),
                              dot(capsule_vertices[1], dir)) + shB.radius;
        auto dist = projA - projB;

        if (dist > distance) {
            distance = dist;
            projection_poly = projA;
            sep_axis = dir;
        }
    }

    if (distance > threshold) {
        return {};
    }

    auto polygon = point_cloud_support_polygon<true>(
        rmeshA.vertices.begin(), rmeshA.vertices.end(), vector3_zero,
        sep_axis, projection_poly, true, support_polygon_tolerance);

    auto is_capsule_edge = std::abs(dot(capsule_vertices[0], sep_axis) -
                                    dot(capsule_vertices[1], sep_axis)) < threshold;

    
}

collision_result collide(const capsule_shape &shA, const polyhedron_shape &shB,
                         const collision_context &ctx) {
    return swap_collide(shA, shB, ctx);
}

}
