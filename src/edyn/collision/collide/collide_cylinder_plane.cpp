#include "edyn/collision/collide.hpp"
#include "edyn/math/math.hpp"

namespace edyn {

void collide(const cylinder_shape &shA, const plane_shape &shB, 
             const collision_context &ctx, collision_result &result) {
    const auto &posA = ctx.posA;
    const auto &ornA = ctx.ornA;
    
    auto normal = shB.normal;
    auto center = normal * shB.constant;

    cylinder_feature featureA;
    size_t feature_indexA;
    vector3 supA;
    scalar projA;
    shA.support_feature(posA, ornA, center, -normal,
                        featureA, feature_indexA, supA, projA,
                        support_feature_tolerance);
    // Flip sign since we're calculating the support point along -normal.
    auto distance = projA *= -1;

    if (distance > ctx.threshold) {
        return;
    }

    switch (featureA) {
    case cylinder_feature::face:{
        auto multipliers = std::array<scalar, 4>{0, 1, 0, -1};
        auto pivotA_x = shA.half_length * to_sign(feature_indexA == 0);

        for(int i = 0; i < 4; ++i) {
            auto pivotA = vector3{pivotA_x,
                                  shA.radius * multipliers[i],
                                  shA.radius * multipliers[(i + 1) % 4]};
            auto pivotA_world = to_world_space(pivotA, posA, ornA);
            auto pivotB = project_plane(pivotA_world, center, normal);
            auto local_distance = dot(pivotA_world - pivotB, normal);
            result.maybe_add_point({pivotA, pivotB, normal, local_distance});
        }
        break;
    }
    case cylinder_feature::side_edge: {
        auto cyl_axis = quaternion_x(ornA);
        auto cyl_vertices = std::array<vector3, 2>{
            posA - cyl_axis * shA.half_length,
            posA + cyl_axis * shA.half_length
        };
        auto dirA = project_direction(-normal, cyl_axis);

        for (auto &vertex : cyl_vertices) {
            auto pivotA_world = vertex + dirA * shA.radius;
            auto pivotA = to_object_space(pivotA_world, posA, ornA);
            auto pivotB = project_plane(pivotA_world, center, normal);
            auto local_distance = dot(pivotA_world - pivotB, normal);
            result.maybe_add_point({pivotA, pivotB, normal, local_distance});
        }
        break;
    }
    case cylinder_feature::cap_edge: {
        auto cyl_axis = quaternion_x(ornA);
        auto vertex = posA + cyl_axis * shA.half_length * to_sign(feature_indexA == 0);
        auto dirA = project_direction(-normal, cyl_axis);
        auto pivotA_world = vertex + dirA * shA.radius;
        auto pivotA = to_object_space(pivotA_world, posA, ornA);
        auto pivotB = project_plane(pivotA_world, center, normal);
        auto local_distance = dot(pivotA_world - pivotB, normal);
        result.maybe_add_point({pivotA, pivotB, normal, local_distance});
        break;
    }
    }
}

void collide(const plane_shape &shA, const cylinder_shape &shB,
             const collision_context &ctx, collision_result &result) {
    swap_collide(shA, shB, ctx, result);
}

}
