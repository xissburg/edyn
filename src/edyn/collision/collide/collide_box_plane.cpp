#include "edyn/collision/collide.hpp"
#include "edyn/math/quaternion.hpp"
#include "edyn/shapes/box_shape.hpp"

namespace edyn {

void collide(const box_shape &shA, const plane_shape &shB, 
             const collision_context &ctx, collision_result &result) {
    auto center = shB.normal * shB.constant;

    box_feature featureA;
    size_t feature_indexA;
    scalar projectionA;
    shA.support_feature(ctx.posA, ctx.ornA, center, -shB.normal, 
                        featureA, feature_indexA, projectionA, 
                        support_feature_tolerance);
    auto distance = -projectionA;

    if (distance > ctx.threshold) {
        return;
    }

    auto vertices = std::array<vector3, 4>{};
    auto num_vertices = size_t{};

    switch (featureA) {
    case box_feature::vertex:
        vertices[0] = shA.get_vertex(feature_indexA);
        num_vertices = 1;
        break;
    case box_feature::edge: {
        auto v = shA.get_edge(feature_indexA);
        vertices[0] = v[0]; vertices[1] = v[1];
        num_vertices = 2;
        break;
    }
    case box_feature::face:
        vertices = shA.get_face(feature_indexA);
        num_vertices = 4;
    }

    for (size_t i = 0; i < num_vertices; ++i) {
        auto &pivotA = vertices[i];
        auto pivotA_world = to_world_space(pivotA, ctx.posA, ctx.ornA);
        auto pivotB_world = project_plane(pivotA_world, center, shB.normal);
        auto pivotB = to_object_space(pivotB_world, ctx.posB, ctx.ornB);
        auto local_distance = dot(pivotA_world - pivotB_world, shB.normal);
        result.add_point({pivotA, pivotB, shB.normal, local_distance});
    }
}

void collide(const plane_shape &shA, const box_shape &shB,
                         const collision_context &ctx, collision_result &result) {
    swap_collide(shA, shB, ctx, result);
}

}
