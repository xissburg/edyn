#include "edyn/shapes/triangle_mesh.hpp"

namespace edyn {

void triangle_mesh::calculate_aabb() {
    aabb.min = vector3_max;
    aabb.max = -vector3_max;

    for (auto &v : vertices) {
        aabb.min = min(aabb.min, v);
        aabb.max = max(aabb.max, v);
    }
}

void triangle_mesh::calculate_adjacency() {
    adjacency.resize(indices.size());
    cos_angles.resize(indices.size());
    is_concave_edge.resize(indices.size());

    // Reset adjacency and angles.
    std::fill(adjacency.begin(), adjacency.end(), UINT16_MAX);
    std::fill(cos_angles.begin(), cos_angles.end(), scalar(-1));
    std::fill(is_concave_edge.begin(), is_concave_edge.end(), false);

    for (size_t i = 0; i < num_triangles(); ++i) {
        // Pointer to first element of the i-th triangle's 3 indices.
        auto i_idx = &indices[i * 3];
        // Normal vector of i-th triangle.
        auto i_edge0 = vertices[i_idx[1]] - vertices[i_idx[0]];
        auto i_edge1 = vertices[i_idx[2]] - vertices[i_idx[1]];
        auto i_normal = cross(i_edge0, i_edge1);
        auto i_normal_len_sqr = length_sqr(i_normal);

        if (i_normal_len_sqr > EDYN_EPSILON) {
            i_normal /= std::sqrt(i_normal_len_sqr);
        }

        // Check all other triangles for shared pairs of vertices.
        for (size_t k = i + 1; k < num_triangles(); ++k) {
            // Pointer to first element of the k-th triangle's 3 indices.
            auto k_idx = &indices[k * 3];
            
            // Check which vertices are shared.
            bool shared_idx[] = {false, false, false};
            auto has_shared_vertex = false;

            for (size_t m = 0; m < 3; ++m) {
                for (size_t n = 0; n < 3; ++n) {
                    if (i_idx[m] == k_idx[n]) {
                        shared_idx[m] = true;
                        has_shared_vertex = true;
                        break;
                    }
                }
            }

            if (!has_shared_vertex) {
                continue;
            }

            // Normal vector of k-th triangle.
            auto k_edge0 = vertices[k_idx[1]] - vertices[k_idx[0]];
            auto k_edge1 = vertices[k_idx[2]] - vertices[k_idx[1]];
            auto k_normal = cross(k_edge0, k_edge1);
            auto k_normal_len_sqr = length_sqr(k_normal);

            if (k_normal_len_sqr > EDYN_EPSILON) {
                k_normal /= std::sqrt(k_normal_len_sqr);
            }

            // Look for pairs of shared indices which translates to a shared edge.
            for (size_t m = 0; m < 3; ++m) {
                auto next_m = (m + 1) % 3;
                if (shared_idx[m] && shared_idx[next_m]) {
                    // Assign triangle k as adjacent to i via edge m.
                    adjacency[i * 3 + m] = k;

                    // Find index of the vertex in triangle k not in edge m.
                    uint16_t other_idx;

                    for (size_t j = 0; j < 3; ++j) {
                        if (k_idx[j] != i_idx[m] && k_idx[j] != i_idx[next_m]) {
                            other_idx = k_idx[j];
                            break;
                        }
                    }

                    // Check if the vertex in triangle k which is not in the shared 
                    // edge is in front of or behind the plane of triangle i.
                    auto concave = dot(i_normal, vertices[other_idx] - vertices[i_idx[m]]) > -EDYN_EPSILON;
                    is_concave_edge[i * 3 + m] = concave;

                    // Get edge angle from the cross product of normals and use 
                    // the sign to classify it as a convex or concave edge.
                    auto cos_angle = dot(i_normal, k_normal);
                    cos_angles[i * 3 + m] = cos_angle;

                    // Find shared edge index in triangle k.
                    size_t n;
                    for (size_t j = 0; j < 3; ++j) {
                        if (k_idx[j] != other_idx && k_idx[(j + 1) % 3] != other_idx) {
                            n = j;
                            break;
                        }
                    }

                    // Assign triangle i as adjacent to k via edge n.
                    adjacency[k * 3 + n] = i;
                    cos_angles[k * 3 + n] = cos_angle;
                    is_concave_edge[k * 3 + n] = concave;

                    // There can be only one shared edge.
                    break;
                }
            }
        }
    }
}

void triangle_mesh::build_tree() {
    std::vector<AABB> aabbs;
    aabbs.reserve(num_triangles());

    for (size_t i = 0; i < num_triangles(); ++i) {
        auto verts = triangle_vertices{
            vertices[indices[i * 3 + 0]],
            vertices[indices[i * 3 + 1]],
            vertices[indices[i * 3 + 2]]
        };

        auto tri_min = min(min(verts[0], verts[1]), verts[2]);
        auto tri_max = max(max(verts[0], verts[1]), verts[2]);

        aabbs.emplace_back(AABB{tri_min, tri_max});
    }

    tree.build(aabbs.begin(), aabbs.end());
}

}