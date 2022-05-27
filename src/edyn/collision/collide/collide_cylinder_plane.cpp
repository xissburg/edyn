#include "edyn/collision/collide.hpp"
#include "edyn/math/math.hpp"
#include "edyn/math/transform.hpp"

namespace edyn {

void collide(const cylinder_shape &shA, const plane_shape &shB,
             const collision_context &ctx, collision_result &result) {
    const auto &posA = ctx.posA;
    const auto &ornA = ctx.ornA;

    auto normal = shB.normal;
    auto center = normal * shB.constant;
    auto projA = -shA.support_projection(posA, ornA, -normal);
    auto projB = shB.constant;
    auto distance = projA - projB;

    if (distance > ctx.threshold) {
        return;
    }

    cylinder_feature featureA;
    size_t feature_indexA;
    shA.support_feature(posA, ornA, -normal, featureA, feature_indexA,
                        support_feature_tolerance);

    collision_result::collision_point point;
    point.normal = normal;
    point.distance = distance;
    point.featureA = {featureA, feature_indexA};
    point.normal_attachment = contact_normal_attachment::normal_on_B;

    switch (featureA) {
    case cylinder_feature::face:{
        auto multipliers = std::array<scalar, 4>{0, 1, 0, -1};
        auto pivotA_axis = shA.half_length * to_sign(feature_indexA == 0);

        // Index of vector element in cylinder object space that represents the
        // cylinder axis followed by the indices of the elements of the axes
        // orthogonal to the cylinder axis.
        auto cyl_ax_idx = static_cast<std::underlying_type_t<coordinate_axis>>(shA.axis);
        auto cyl_ax_idx_ortho0 = (cyl_ax_idx + 1) % 3;
        auto cyl_ax_idx_ortho1 = (cyl_ax_idx + 2) % 3;

        for(int i = 0; i < 4; ++i) {
            point.pivotA[cyl_ax_idx] = pivotA_axis;
            point.pivotA[cyl_ax_idx_ortho0] = shA.radius * multipliers[i];
            point.pivotA[cyl_ax_idx_ortho1] = shA.radius * multipliers[(i + 1) % 4];
            auto pivotA_world = to_world_space(point.pivotA, posA, ornA);
            point.pivotB = project_plane(pivotA_world, center, normal);
            point.distance = dot(pivotA_world - point.pivotB, normal);
            result.maybe_add_point(point);
        }
        break;
    }
    case cylinder_feature::side_edge:
    case cylinder_feature::cap_edge: {
        auto cyl_axis = coordinate_axis_vector(shA.axis, ornA);
        auto cyl_vertices = std::array<vector3, 2>{};
        auto num_vertices = 0;

        if (featureA == cylinder_feature::cap_edge) {
            cyl_vertices[0] = posA + cyl_axis * shA.half_length * to_sign(feature_indexA == 0);
            num_vertices = 1;
        } else {
            cyl_vertices[0] = posA - cyl_axis * shA.half_length;
            cyl_vertices[1] = posA + cyl_axis * shA.half_length;
            num_vertices = 2;
        }

        // Negated normal projected onto plane of cylinder caps. Multiply it by
        // radius and add the cap center to get the exact pivot on A.
        auto dirA = normalize(project_direction(-normal, cyl_axis));

        for (auto i = 0; i < num_vertices; ++i) {
            auto &vertex = cyl_vertices[i];
            auto pivotA_world = vertex + dirA * shA.radius;
            point.pivotA = to_object_space(pivotA_world, posA, ornA);
            point.pivotB = project_plane(pivotA_world, center, normal);
            point.distance = dot(pivotA_world - point.pivotB, normal);
            result.maybe_add_point(point);
        }
        break;
    }
    }
}

void collide(const plane_shape &shA, const cylinder_shape &shB,
             const collision_context &ctx, collision_result &result) {
    swap_collide(shA, shB, ctx, result);
}

}
