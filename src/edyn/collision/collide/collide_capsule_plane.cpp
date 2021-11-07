#include "edyn/collision/collide.hpp"
#include "edyn/math/transform.hpp"

namespace edyn {

void collide(const capsule_shape &shA, const plane_shape &shB,
             const collision_context &ctx, collision_result &result) {
    auto center = shB.normal * shB.constant;
    auto capsule_vertices = shA.get_vertices(ctx.posA, ctx.ornA);

    scalar proj_capsule_vertices[] = {
        dot(capsule_vertices[0] - center, shB.normal),
        dot(capsule_vertices[1] - center, shB.normal)
    };

    collision_feature featureA;
    auto is_capsule_edge = std::abs(proj_capsule_vertices[0] -
                                    proj_capsule_vertices[1]) < support_feature_tolerance;

    if (is_capsule_edge) {
        featureA.feature = capsule_feature::side;
    } else {
        featureA.feature = capsule_feature::hemisphere;
        featureA.index = proj_capsule_vertices[0] < proj_capsule_vertices[1] ? 0 : 1;
    }

    for (auto i = 0; i < 2; ++i) {
        auto distance = proj_capsule_vertices[i] - shA.radius;

        if (distance > ctx.threshold) continue;

        auto vertex = capsule_vertices[i];
        auto pivotA_world = vertex - shB.normal * shA.radius;
        auto pivotA = to_object_space(pivotA_world, ctx.posA, ctx.ornA);
        auto pivotB = project_plane(vertex, center, shB.normal);
        result.add_point({pivotA, pivotB, shB.normal, distance, contact_normal_attachment::normal_on_B, featureA});
    }
}

void collide(const plane_shape &shA, const capsule_shape &shB,
             const collision_context &ctx, collision_result &result) {
    swap_collide(shA, shB, ctx, result);
}

}
