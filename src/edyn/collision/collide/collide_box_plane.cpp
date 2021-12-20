#include "edyn/collision/collide.hpp"
#include "edyn/math/transform.hpp"
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

    collision_result::collision_point point;
    point.normal = shB.normal;
    point.distance = distance;
    point.featureA = {featureA, feature_indexA};
    point.normal_attachment = contact_normal_attachment::normal_on_B;

    for (size_t i = 0; i < num_vertices; ++i) {
        point.pivotA = vertices[i];
        auto pivotA_world = to_world_space(point.pivotA, ctx.posA, ctx.ornA);
        auto pivotB_world = project_plane(pivotA_world, center, shB.normal);
        point.pivotB = to_object_space(pivotB_world, ctx.posB, ctx.ornB);
        point.distance = dot(pivotA_world - pivotB_world, shB.normal);
        result.add_point(point);
    }
}

void collide(const plane_shape &shA, const box_shape &shB,
             const collision_context &ctx, collision_result &result) {
    swap_collide(shA, shB, ctx, result);
}

}
