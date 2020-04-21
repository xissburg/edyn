#include "edyn/collision/collide.hpp"

namespace edyn {

struct box_mesh_separating_axis {
    vector3 dir;
    box_feature featureA;
    triangle_feature featureB;
    uint8_t feature_indexA;
    uint8_t feature_indexB;
    scalar distance;
};

collision_result collide(const box_shape &shA, const vector3 &posA, const quaternion &ornA,
                         const mesh_shape &shB, const vector3 &posB, const quaternion &ornB,
                         scalar threshold) {
    auto result = collision_result{};

    // Box position and orientation in mesh's space.
    const auto ornB_conj = conjugate(ornB);
    const auto posA_in_B = rotate(ornB_conj, posA - posB);
    const auto ornA_in_B = ornB_conj * ornA;

    const auto axesA = std::array<vector3, 3>{
        quaternion_x(ornA_in_B),
        quaternion_y(ornA_in_B),
        quaternion_z(ornA_in_B)
    };


    auto aabb = shA.aabb(posA_in_B, ornA_in_B);
    shB.trimesh->visit(aabb, [&] (size_t tri_idx, const triangle_vertices &vertices) {
        if (result.num_points == max_contacts) {
            return;
        }

        const auto edges = get_triangle_edges(vertices);
        const auto tri_normal = normalize(cross(edges[0], edges[1]));

        std::array<box_mesh_separating_axis, 13> sep_axes;
        uint8_t axis_idx = 0;

        // Box faces.
        for (uint8_t i = 0; i < 3; ++i) {
            auto &axisA = axesA[i];
            auto &axis = sep_axes[axis_idx++];
            axis.featureA = BOX_FEATURE_FACE;

            // Find which direction gives greatest penetration.
            triangle_feature neg_tri_feature, pos_tri_feature;
            uint8_t neg_tri_feature_index, pos_tri_feature_index;
            scalar neg_tri_proj, pos_tri_proj;
            get_triangle_support_feature(vertices, posA_in_B, -axisA, neg_tri_feature, neg_tri_feature_index, neg_tri_proj);
            get_triangle_support_feature(vertices, posA_in_B, axisA, pos_tri_feature, pos_tri_feature_index, pos_tri_proj);

            if (neg_tri_proj < pos_tri_proj) {
                axis.dir = -axisA;
                axis.feature_indexA = i * 2;
                axis.featureB = neg_tri_feature;
                axis.feature_indexB = neg_tri_feature_index;
                axis.distance = -(shA.half_extents[i] + neg_tri_proj);
            } else {
                axis.dir = axisA;
                axis.feature_indexA = i * 2 + 1;
                axis.featureB = pos_tri_feature;
                axis.feature_indexB = pos_tri_feature_index;
                axis.distance = -(shA.half_extents[i] + pos_tri_proj);
            }
        }

        // Triangle face normal.
        {
            const auto &v0 = vertices[0];
            auto &axis = sep_axes[axis_idx++];
            axis.featureB = TRIANGLE_FEATURE_FACE;
            axis.dir = tri_normal;

            shA.support_feature(posA_in_B, ornA_in_B, 
                                vertices[0], -tri_normal, 
                                axis.featureA, axis.feature_indexA, 
                                axis.distance);
            // Make distance negative when penetrating.
            axis.distance *= -1;
        }

        // Edges.
        for (uint8_t i = 0; i < 3; ++i) {
            auto &axisA = axesA[i];

            for (uint8_t j = 0; j < 3; ++j) {
                auto &axisB = edges[j];
                auto &axis = sep_axes[axis_idx];
                axis.dir = cross(axisA, axisB);
                auto dir_len_sqr = length2(axis.dir);

                if (dir_len_sqr <= EDYN_EPSILON) {
                    continue;
                }

                axis.dir /= std::sqrt(dir_len_sqr);

                if (dot(posA_in_B - vertices[j], axis.dir) < 0) {
                    // Make it point towards A.
                    axis.dir *= -1;
                }

                scalar projA, projB;
                shA.support_feature(posA_in_B, ornA_in_B, vertices[j], -axis.dir, axis.featureA, axis.feature_indexA, projA);
                get_triangle_support_feature(vertices, vertices[j], axis.dir, axis.featureB, axis.feature_indexB, projB);
                axis.distance = -(projA + projB);

                ++axis_idx;
            }
        }

        // Get axis with greatest penetration.
        auto greatest_distance = -EDYN_SCALAR_MAX;
        uint8_t sep_axis_idx;

        for (uint8_t i = 0; i < axis_idx; ++i) {
            auto &sep_axis = sep_axes[i];
            
            if (sep_axis.distance > greatest_distance) {
                greatest_distance = sep_axis.distance;
                sep_axis_idx = i;
            }
        }

        auto &sep_axis = sep_axes[sep_axis_idx];

        if (sep_axis.distance > threshold) {
            return;
        }

        if (sep_axis.featureA == BOX_FEATURE_EDGE && sep_axis.featureB == TRIANGLE_FEATURE_EDGE) {
            scalar s[2], t[2];
            vector3 p0[2], p1[2];
            size_t num_points = 0;
            auto edgeA = shA.get_edge(sep_axis.feature_indexA, posA_in_B, ornA_in_B);
            vector3 edgeB[2];
            edgeB[0] = vertices[sep_axis.feature_indexB];
            edgeB[1] = vertices[(sep_axis.feature_indexB + 1) % 3];
            closest_point_segment_segment(edgeA[0], edgeA[1], edgeB[0], edgeB[1], 
                                          s[0], t[0], p0[0], p1[0], &num_points, 
                                          &s[1], &t[1], &p0[1], &p1[1]);

            for (uint8_t i = 0; i < num_points; ++i) {
                if (s[i] > 0 && s[i] < 1 && t[i] > 0 && t[i] < 1) {
                    auto idx = result.num_points++;
                    result.point[idx].pivotA = to_object_space(p0[i], posA, ornA);
                    result.point[idx].pivotB = to_object_space(p1[i], posB, ornB);
                    result.point[idx].normalB = sep_axis.dir;
                    result.point[idx].distance = sep_axis.distance;
                }
            }
        }

        switch (sep_axis.featureA) {
        case BOX_FEATURE_FACE:
            switch (sep_axis.featureB) {
            case TRIANGLE_FEATURE_VERTEX: {
                auto vertex = vertices[sep_axis.feature_indexB];
                auto vertex_proj = vertex - sep_axis.dir * sep_axis.distance;
                auto vertex_proj_world = posB + rotate(ornB, vertex);
                auto vertex_proj_in_A = rotate(conjugate(ornA), vertex_proj_world - posA);

                auto idx = result.num_points++;
                result.point[idx].pivotA = vertex_proj_in_A;
                result.point[idx].pivotB = vertex;
                result.point[idx].normalB = sep_axis.dir;
                result.point[idx].distance = sep_axis.distance;
                break;
            }
            case TRIANGLE_FEATURE_EDGE: {

                break;
            }
            }
            break;

        case BOX_FEATURE_EDGE:
            break;

        case BOX_FEATURE_VERTEX:
            switch (sep_axis.featureB) {
                case TRIANGLE_FEATURE_FACE: {
                    auto pivotA = shA.get_vertex(sep_axis.feature_indexA);
                    auto pivotB = posA_in_B + rotate(ornA_in_B, pivotA) - tri_normal * sep_axis.distance;

                    auto idx = result.num_points++;
                    result.point[idx].pivotA = pivotA;
                    result.point[idx].pivotB = pivotB;
                    result.point[idx].normalB = sep_axis.dir;
                    result.point[idx].distance = sep_axis.distance;
                    break;
                }
            }
        }
    });

    return result;
}

}