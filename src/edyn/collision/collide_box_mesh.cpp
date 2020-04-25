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

static
bool point_in_quad(const vector3 &p, 
                   const std::array<vector3, 4> &quad_vertices, 
                   const vector3 &quad_normal) {

    std::array<vector3, 4> quad_tangents;
    for (int i = 0; i < 4; ++i) {
        auto &v0 = quad_vertices[i];
        auto &v1 = quad_vertices[(i + 1) % 4];
        quad_tangents[i] = cross(quad_normal, v1 - v0);
    }

    scalar dots[4];
    for (int i = 0; i < 4; ++i) {
        dots[i] = dot(p - quad_vertices[i], quad_tangents[i]);
    }

    return dots[0] > -EDYN_EPSILON && dots[1] > -EDYN_EPSILON &&
           dots[2] > -EDYN_EPSILON && dots[3] > -EDYN_EPSILON;
}

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
        const auto edges = get_triangle_edges(vertices);
        const auto tri_normal = normalize(cross(edges[0], edges[1]));
        bool is_concave_edge[3];
        bool is_concave_vertex[3];

        for (int i = 0; i < 3; ++i) {
            is_concave_edge[i] = shB.trimesh->is_concave_edge[tri_idx * 3 + i];
        }

        for (int i = 0; i < 3; ++i) {
            // If edge starting or ending in this vertex are concave then thus is the vertex.
            is_concave_vertex[i] = is_concave_edge[i] || is_concave_edge[(i + 2) % 3];
        }

        vector3 edge_tangents[3];

        for (int i = 0; i < 3; ++i) {
            edge_tangents[i] = cross(edges[i], tri_normal);
        }

        std::array<box_mesh_separating_axis, 13> sep_axes;
        uint8_t axis_idx = 0;

        // Box faces.
        for (uint8_t i = 0; i < 3; ++i) {
            auto &axisA = axesA[i];
            auto &axis = sep_axes[axis_idx];
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

            if (axis.featureB == TRIANGLE_FEATURE_VERTEX) {
                if (!is_concave_vertex[axis.feature_indexB]) {
                    ++axis_idx;
                }
            } else if (axis.featureB == TRIANGLE_FEATURE_EDGE) {
                    // Ignore concave edges.
                if (!is_concave_edge[axis.feature_indexB]) {
                    // Direction must be in the edge's Voronoi region.
                    if (dot(axis.dir, edge_tangents[axis.feature_indexB]) > 0 &&
                        dot(axis.dir, tri_normal) > shB.trimesh->cos_angles[tri_idx * 3 + axis.feature_indexB]) {
                        ++axis_idx;
                    }
                }
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
                if (is_concave_edge[j]) {
                    continue;
                }

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

                // Support feature must be the current edge.
                if (axis.featureB == TRIANGLE_FEATURE_EDGE && axis.feature_indexB == j) {
                    // Ignore concave edges.
                    if (!is_concave_edge[j]) {
                        // Direction must be in the edge's Voronoi region.
                        if (dot(axis.dir, edge_tangents[j]) > 0 &&
                            dot(axis.dir, tri_normal) > shB.trimesh->cos_angles[tri_idx * 3 + j]) {
                            ++axis_idx;
                        }
                    }
                }
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

        // No collision.
        if (sep_axis.distance > threshold) {
            return;
        }

        if (sep_axis.featureA == BOX_FEATURE_FACE && sep_axis.featureB == TRIANGLE_FEATURE_FACE) {
            auto face_normal_in_B = shA.get_face_normal(sep_axis.feature_indexA, ornA_in_B);
            auto face_vertices = shA.get_face(sep_axis.feature_indexA);
            std::array<vector3, 4> face_vertices_in_B;
            for (int i = 0; i < 4; ++i) {
                face_vertices_in_B[i] = posA_in_B + rotate(ornA_in_B, face_vertices[i]);
            }

            size_t num_tri_vert_in_box_face = 0;

            // Check for triangle vertices inside box face.
            for (int i = 0; i < 3; ++i) {
                // Ignore vertices that are on a concave edge.
                if (is_concave_vertex[i]) {
                    continue;
                }

                if (point_in_quad(vertices[i], face_vertices_in_B, face_normal_in_B)) {
                    // Triangle vertex is inside box face.
                    auto pivot_face = project_plane(vertices[i], face_vertices_in_B[0], sep_axis.dir);
                    auto pivot_face_world = posB + rotate(ornB, pivot_face);
                    auto pivotA = to_object_space(pivot_face_world, posA, ornA);
                    auto pivotB = vertices[i];
                    result.add_point({pivotA, pivotB, sep_axis.dir, sep_axis.distance});
                    ++num_tri_vert_in_box_face;
                }
            }

            // Continue if not all triangle vertices are contained in face.
            size_t num_box_vert_in_tri_face = 0;

            if (num_tri_vert_in_box_face < 3) {
                // Look for box face vertices inside triangle face.
                for (int i = 0; i < 4; ++i) {
                    if (point_in_triangle(vertices, tri_normal, face_vertices_in_B[i])) {
                        auto pivotA = face_vertices[i];
                        auto pivotB = project_plane(face_vertices_in_B[i], vertices[0], sep_axis.dir);
                        result.add_point({pivotA, pivotB, sep_axis.dir, sep_axis.distance});
                        ++num_box_vert_in_tri_face;
                    }
                }
            }

            // Continue if not all box's face vertices are contained in the triangle.
            // Perform edge intersection tests.
            if (num_box_vert_in_tri_face < 4) {
                for (int i = 0; i < 4; ++i) {
                    auto &a0 = face_vertices_in_B[i];
                    auto &a1 = face_vertices_in_B[(i + 1) % 4];

                    for (int j = 0; j < 3; ++j) {
                        // Ignore concave edges.
                        if (is_concave_edge[j]) {
                            continue;
                        }

                        auto &b0 = vertices[j];
                        auto &b1 = vertices[(j + 1) % 3];

                        scalar s[2], t[2];
                        vector3 p0[2], p1[2];
                        size_t num_points = 0;
                        closest_point_segment_segment(a0, a1, b0, b1, 
                                                      s[0], t[0], p0[0], p1[0], &num_points, 
                                                      &s[1], &t[1], &p0[1], &p1[1]);
                        for (uint8_t k = 0; k < num_points; ++k) {
                            if (s[k] > 0 && s[k] < 1 && t[k] > 0 && t[k] < 1) {
                                auto p0_world = posB + rotate(ornB, p0[k]);
                                auto pivotA = to_object_space(p0_world, posA, ornA);
                                auto pivotB = p1[k];
                                result.add_point({pivotA, pivotB, sep_axis.dir, sep_axis.distance});
                            }
                        }
                    }
                }
            }
        } else if (sep_axis.featureA == BOX_FEATURE_FACE && sep_axis.featureB == TRIANGLE_FEATURE_EDGE) {
            EDYN_ASSERT(!is_concave_edge[sep_axis.feature_indexB]);
        
            auto face_normal = shA.get_face_normal(sep_axis.feature_indexA, ornA_in_B);
            auto face_vertices = shA.get_face(sep_axis.feature_indexA, posA_in_B, ornA_in_B);

            // Check if edge vertices are inside box face.
            vector3 edge_vertices[] = {vertices[sep_axis.feature_indexB],
                                       vertices[(sep_axis.feature_indexB + 1) % 3]};
            size_t num_edge_vert_in_box_face = 0;

            for (int i = 0; i < 2; ++i) {
                if (point_in_quad(edge_vertices[i], face_vertices, face_normal)) {
                    // Edge's vertex is inside face.
                    auto pivot_face = project_plane(edge_vertices[i], face_vertices[0], face_normal);
                    auto pivot_face_world = posB + rotate(ornB, pivot_face);
                    auto pivotA = to_object_space(pivot_face_world, posA, ornA);
                    auto pivotB = edge_vertices[i];
                    result.add_point({pivotA, pivotB, sep_axis.dir, sep_axis.distance});
                    ++num_edge_vert_in_box_face;
                }
            }

            // If both vertices are not inside the face then perform edge intersection tests.
            if (num_edge_vert_in_box_face < 2) {
                for (int i = 0; i < 4; ++i) {
                    auto &v0 = face_vertices[i];
                    auto &v1 = face_vertices[(i + 1) % 4];

                    scalar s[2], t[2];
                    vector3 p0[2], p1[2];
                    size_t num_points = 0;
                    closest_point_segment_segment(v0, v1, edge_vertices[0], edge_vertices[1], 
                                                s[0], t[0], p0[0], p1[0], &num_points, 
                                                &s[1], &t[1], &p0[1], &p1[1]);
                    for (int i = 0; i < num_points; ++i) {
                        if (s[i] > 0 && s[i] < 1 && t[i] > 0 && t[i] < 1) {
                            auto p0_world = posB + rotate(ornB, p0[i]);
                            auto pivotA = to_object_space(p0_world, posA, ornA);
                            auto pivotB = p1[i];
                            result.add_point({pivotA, pivotB, sep_axis.dir, sep_axis.distance});
                        }
                    }
                }
            }
        } else if (sep_axis.featureA == BOX_FEATURE_EDGE && sep_axis.featureB == TRIANGLE_FEATURE_FACE) {
            // Check if edge vertices are inside triangle face.
            auto edge = shA.get_edge(sep_axis.feature_indexA);
            auto edge_in_B = edge;
            size_t num_edge_vert_in_tri_face = 0;

            for (size_t i = 0; i < 2; ++i) {
                edge_in_B[i] = posA_in_B + rotate(ornA_in_B, edge[i]);

                if (point_in_triangle(vertices, tri_normal, edge_in_B[i])) {
                    auto pivotA = edge[i];
                    auto pivotB = project_plane(edge_in_B[i], vertices[0], sep_axis.dir);
                    result.add_point({pivotA, pivotB, sep_axis.dir, sep_axis.distance});
                    num_edge_vert_in_tri_face = 0;
                }
            }

            // If both vertices are not inside the face then perform edge intersection tests.
            if (num_edge_vert_in_tri_face < 2) {
                for (int i = 0; i < 3; ++i) {
                    // Ignore concave edges.
                    if (is_concave_edge[i]) {
                        continue;
                    }
                    
                    auto &v0 = vertices[i];
                    auto &v1 = vertices[(i + 1) % 3];

                    scalar s[2], t[2];
                    vector3 p0[2], p1[2];
                    size_t num_points = 0;
                    closest_point_segment_segment(edge_in_B[0], edge_in_B[1], v0, v1, 
                                                  s[0], t[0], p0[0], p1[0], &num_points, 
                                                  &s[1], &t[1], &p0[1], &p1[1]);
                    for (int i = 0; i < num_points; ++i) {
                        if (s[i] > 0 && s[i] < 1 && t[i] > 0 && t[i] < 1) {
                            auto p0_world = posB + rotate(ornB, p0[i]);
                            auto pivotA = to_object_space(p0_world, posA, ornA);
                            auto pivotB = p1[i];
                            result.add_point({pivotA, pivotB, sep_axis.dir, sep_axis.distance});
                        }
                    }
                }
            }
        } else if (sep_axis.featureA == BOX_FEATURE_EDGE && sep_axis.featureB == TRIANGLE_FEATURE_EDGE) {
            EDYN_ASSERT(!is_concave_edge[sep_axis.feature_indexB]);
                
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

            for (int i = 0; i < num_points; ++i) {
                if (s[i] > 0 && s[i] < 1 && t[i] > 0 && t[i] < 1) {
                    auto pivotA = to_object_space(p0[i], posA, ornA);
                    auto pivotB = to_object_space(p1[i], posB, ornB);
                    result.add_point({pivotA, pivotB, sep_axis.dir, sep_axis.distance});
                }
            }
        } else if (sep_axis.featureA == BOX_FEATURE_FACE && sep_axis.featureB == TRIANGLE_FEATURE_VERTEX) {
            // Ignore vertices that are on a concave edge.
            EDYN_ASSERT(!is_concave_vertex[sep_axis.feature_indexB]);
            auto vertex = vertices[sep_axis.feature_indexB];
            auto face_normal = shA.get_face_normal(sep_axis.feature_indexA, ornA_in_B);
            auto face_vertices = shA.get_face(sep_axis.feature_indexA, posA_in_B, ornA_in_B);

            if (point_in_quad(vertex, face_vertices, face_normal)) {
                auto vertex_proj = vertex - sep_axis.dir * sep_axis.distance;
                auto vertex_proj_world = posB + rotate(ornB, vertex);
                auto pivotA = to_object_space(vertex_proj_world, posA, ornA);
                auto pivotB = vertex;
                result.add_point({pivotA, pivotB, sep_axis.dir, sep_axis.distance});
            }
        } else if (sep_axis.featureA == BOX_FEATURE_VERTEX && sep_axis.featureB == TRIANGLE_FEATURE_FACE) {
            auto pivotA = shA.get_vertex(sep_axis.feature_indexA);
            auto pivotB = posA_in_B + rotate(ornA_in_B, pivotA) - tri_normal * sep_axis.distance;
            if (point_in_triangle(vertices, tri_normal, pivotB)) {
                result.add_point({pivotA, pivotB, sep_axis.dir, sep_axis.distance});
            }
        }
    });

    return result;
}

}