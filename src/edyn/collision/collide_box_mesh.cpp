#include "edyn/collision/collide.hpp"
#include "edyn/math/math.hpp"

namespace edyn {

struct box_mesh_separating_axis {
    vector3 dir;
    box_feature featureA;
    triangle_feature featureB;
    size_t feature_indexA;
    size_t feature_indexB;
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
        size_t axis_idx = 0;

        // Box faces.
        for (size_t i = 0; i < 3; ++i) {
            auto &axisA = axesA[i];
            auto &axis = sep_axes[axis_idx];
            axis.featureA = BOX_FEATURE_FACE;

            // Find which direction gives greatest penetration.
            triangle_feature neg_tri_feature, pos_tri_feature;
            size_t neg_tri_feature_index, pos_tri_feature_index;
            scalar neg_tri_proj, pos_tri_proj;
            get_triangle_support_feature(vertices, posA_in_B, -axisA, 
                                         neg_tri_feature, neg_tri_feature_index, 
                                         neg_tri_proj, threshold);
            get_triangle_support_feature(vertices, posA_in_B, axisA, 
                                         pos_tri_feature, pos_tri_feature_index, 
                                         pos_tri_proj, threshold);

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
                                axis.distance, threshold);
            // Make distance negative when penetrating.
            axis.distance *= -1;
        }

        // Edges.
        for (size_t i = 0; i < 3; ++i) {
            auto &axisA = axesA[i];

            for (size_t j = 0; j < 3; ++j) {
                if (is_concave_edge[j]) {
                    continue;
                }

                auto &axisB = edges[j];
                auto &axis = sep_axes[axis_idx];
                axis.dir = cross(axisA, axisB);
                auto dir_len_sqr = length_sqr(axis.dir);

                if (dir_len_sqr <= EDYN_EPSILON) {
                    continue;
                }

                axis.dir /= std::sqrt(dir_len_sqr);

                if (dot(posA_in_B - vertices[j], axis.dir) < 0) {
                    // Make it point towards A.
                    axis.dir *= -1;
                }

                scalar projA, projB;
                shA.support_feature(posA_in_B, ornA_in_B, vertices[j], -axis.dir, 
                                    axis.featureA, axis.feature_indexA, 
                                    projA, threshold);
                get_triangle_support_feature(vertices, vertices[j], axis.dir, 
                                             axis.featureB, axis.feature_indexB, 
                                             projB, threshold);
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

        // Get axis with greatest distance.
        auto greatest_distance = -EDYN_SCALAR_MAX;
        size_t sep_axis_idx;

        for (size_t i = 0; i < axis_idx; ++i) {
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
                    auto pivotA = to_object_space(pivot_face, posA_in_B, ornA_in_B);
                    auto pivotB = vertices[i];
                    result.maybe_add_point({pivotA, pivotB, sep_axis.dir, sep_axis.distance});
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
                        result.maybe_add_point({pivotA, pivotB, sep_axis.dir, sep_axis.distance});
                        ++num_box_vert_in_tri_face;
                    }
                }
            }

            // Continue if not all box's face vertices are contained in the triangle.
            // Perform edge intersection tests.
            if (num_box_vert_in_tri_face < 4) {
                for (int i = 0; i < 4; ++i) {
                    auto &a0_in_B = face_vertices_in_B[i];
                    auto &a1_in_B = face_vertices_in_B[(i + 1) % 4];
                    
                    auto &a0 = face_vertices[i];
                    auto &a1 = face_vertices[(i + 1) % 4];

                    for (int j = 0; j < 3; ++j) {
                        // Ignore concave edges.
                        if (is_concave_edge[j]) {
                            continue;
                        }

                        auto &b0 = vertices[j];
                        auto &b1 = vertices[(j + 1) % 3];

                        // Convert this into a 2D segment intersection problem in the box' space.
                        auto b0_in_A = to_object_space(b0, posA_in_B, ornA_in_B);
                        auto b1_in_A = to_object_space(b1, posA_in_B, ornA_in_B);
                        
                        vector2 p0, p1, q0, q1;

                        if (sep_axis.feature_indexA == 0 || sep_axis.feature_indexA == 1) { // X face
                            p0 = {a0.z, a0.y}; p1 = {a1.z, a1.y};
                            q0 = {b0_in_A.z, b0_in_A.y}; q1 = {b1_in_A.z, b1_in_A.y};
                        } else if (sep_axis.feature_indexA == 2 || sep_axis.feature_indexA == 3) { // Y face
                            p0 = {a0.x, a0.z}; p1 = {a1.x, a1.z};
                            q0 = {b0_in_A.x, b0_in_A.z}; q1 = {b1_in_A.x, b1_in_A.z};
                        } else { // if (sep_axis.feature_indexA == 4 || sep_axis.feature_indexA == 5) { // Z face
                            p0 = {a0.x, a0.y}; p1 = {a1.x, a1.y};
                            q0 = {b0_in_A.x, b0_in_A.y}; q1 = {b1_in_A.x, b1_in_A.y};
                        }

                        scalar s[2], t[2];
                        auto num_points = intersect_segments(p0, p1, q0, q1, s[0], t[0], s[1], t[1]);

                        for (size_t k = 0; k < num_points; ++k) {
                            auto pivotA = lerp(a0, a1, s[k]);
                            auto pivotB = lerp(b0, b1, t[k]);
                            result.maybe_add_point({pivotA, pivotB, sep_axis.dir, sep_axis.distance});
                        }
                    }
                }
            }
        } else if (sep_axis.featureA == BOX_FEATURE_FACE && sep_axis.featureB == TRIANGLE_FEATURE_EDGE) {
            EDYN_ASSERT(!is_concave_edge[sep_axis.feature_indexB]);
        
            auto face_normal_in_B = shA.get_face_normal(sep_axis.feature_indexA, ornA_in_B);
            auto face_vertices = shA.get_face(sep_axis.feature_indexA);
            std::array<vector3, 4> face_vertices_in_B;
            for (int i = 0; i < 4; ++i) {
                face_vertices_in_B[i] = posA_in_B + rotate(ornA_in_B, face_vertices[i]);
            }

            // Check if edge vertices are inside box face.
            vector3 edge_vertices[] = {vertices[sep_axis.feature_indexB],
                                       vertices[(sep_axis.feature_indexB + 1) % 3]};
            size_t num_edge_vert_in_box_face = 0;

            for (int i = 0; i < 2; ++i) {
                if (point_in_quad(edge_vertices[i], face_vertices_in_B, face_normal_in_B)) {
                    // Edge's vertex is inside face.
                    auto pivot_face = project_plane(edge_vertices[i], face_vertices_in_B[0], face_normal_in_B);
                    auto pivotA = to_object_space(pivot_face, posA_in_B, ornA_in_B);
                    auto pivotB = edge_vertices[i];
                    result.maybe_add_point({pivotA, pivotB, sep_axis.dir, sep_axis.distance});
                    ++num_edge_vert_in_box_face;
                }
            }

            // If both vertices are not inside the face then perform edge intersection tests.
            if (num_edge_vert_in_box_face < 2) {
                for (int i = 0; i < 4; ++i) {
                    auto &a0 = face_vertices[i];
                    auto &a1 = face_vertices[(i + 1) % 4];
                    auto e0_in_A = to_object_space(edge_vertices[0], posA_in_B, ornA_in_B);
                    auto e1_in_A = to_object_space(edge_vertices[1], posA_in_B, ornA_in_B);
                    
                    vector2 p0, p1, q0, q1;

                    if (sep_axis.feature_indexA == 0 || sep_axis.feature_indexA == 1) { // X face
                        p0 = {a0.z, a0.y}; p1 = {a1.z, a1.y};
                        q0 = {e0_in_A.z, e0_in_A.y}; q1 = {e1_in_A.z, e1_in_A.y};
                    } else if (sep_axis.feature_indexA == 2 || sep_axis.feature_indexA == 3) { // Y face
                        p0 = {a0.x, a0.z}; p1 = {a1.x, a1.z};
                        q0 = {e0_in_A.x, e0_in_A.z}; q1 = {e1_in_A.x, e1_in_A.z};
                    } else { // if (sep_axis.feature_indexA == 4 || sep_axis.feature_indexA == 5) { // Z face
                        p0 = {a0.x, a0.y}; p1 = {a1.x, a1.y};
                        q0 = {e0_in_A.x, e0_in_A.y}; q1 = {e1_in_A.x, e1_in_A.y};
                    }

                    scalar s[2], t[2];
                    auto num_points = intersect_segments(p0, p1, q0, q1, s[0], t[0], s[1], t[1]);

                    for (size_t k = 0; k < num_points; ++k) {
                        auto pivotA = lerp(a0, a1, s[k]);
                        auto pivotB = lerp(edge_vertices[0], edge_vertices[1], t[k]);
                        result.maybe_add_point({pivotA, pivotB, sep_axis.dir, sep_axis.distance});
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
                    result.maybe_add_point({pivotA, pivotB, sep_axis.dir, sep_axis.distance});
                    num_edge_vert_in_tri_face = 0;
                }
            }

            // If both vertices are not inside the face then perform segment intersections.
            if (num_edge_vert_in_tri_face < 2) {
                auto &tri_origin = vertices[0];
                auto tangent = normalize(vertices[1] - vertices[0]);
                auto bitangent = cross(tri_normal, tangent);
                auto tri_basis = matrix3x3_columns(tangent, tri_normal, bitangent);

                auto e0_in_tri = (edge_in_B[0] - tri_origin) * tri_basis;
                auto e1_in_tri = (edge_in_B[1] - tri_origin) * tri_basis;
                auto p0 = vector2{e0_in_tri.x, e0_in_tri.z};
                auto p1 = vector2{e1_in_tri.x, e1_in_tri.z};

                for (int i = 0; i < 3; ++i) {
                    // Ignore concave edges.
                    if (is_concave_edge[i]) {
                        continue;
                    }
                    
                    auto &v0 = vertices[i];
                    auto &v1 = vertices[(i + 1) % 3];

                    auto v0_in_tri = (v0 - tri_origin) * tri_basis; // multiply by transpose.
                    auto v1_in_tri = (v1 - tri_origin) * tri_basis;

                    auto q0 = vector2{v0_in_tri.x, v0_in_tri.z};
                    auto q1 = vector2{v1_in_tri.x, v1_in_tri.z};

                    scalar s[2], t[2];
                    auto num_points = intersect_segments(p0, p1, q0, q1, s[0], t[0], s[1], t[1]);

                    for (size_t k = 0; k < num_points; ++k) {
                        auto pivotA = lerp(edge[0], edge[1], s[k]);
                        auto pivotB = lerp(v0, v1, t[k]);
                        result.maybe_add_point({pivotA, pivotB, sep_axis.dir, sep_axis.distance});
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
                    result.maybe_add_point({pivotA, pivotB, sep_axis.dir, sep_axis.distance});
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
                result.maybe_add_point({pivotA, pivotB, sep_axis.dir, sep_axis.distance});
            }
        } else if (sep_axis.featureA == BOX_FEATURE_VERTEX && sep_axis.featureB == TRIANGLE_FEATURE_FACE) {
            auto pivotA = shA.get_vertex(sep_axis.feature_indexA);
            auto pivotB = posA_in_B + rotate(ornA_in_B, pivotA) - tri_normal * sep_axis.distance;
            if (point_in_triangle(vertices, tri_normal, pivotB)) {
                result.maybe_add_point({pivotA, pivotB, sep_axis.dir, sep_axis.distance});
            }
        }
    });

    return result;
}

}