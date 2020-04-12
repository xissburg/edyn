#include "edyn/shapes/triangle_mesh.hpp"

namespace edyn {

void triangle_mesh::calculate_aabb() {
    aabb.min = vector3_max;
    aabb.max = -vector3_max;

    for (auto &v : vertices) {
        if (v.x < aabb.min.x) {
            aabb.min.x = v.x;
        }
        if (v.y < aabb.min.y) {
            aabb.min.y = v.y;
        }
        if (v.z < aabb.min.z) {
            aabb.min.z = v.z;
        }
        if (v.x > aabb.max.x) {
            aabb.max.x = v.x;
        }
        if (v.y > aabb.max.y) {
            aabb.max.y = v.y;
        }
        if (v.z > aabb.max.z) {
            aabb.max.z = v.z;
        }
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
        auto i_normal = normalize(cross(vertices[i_idx[1]] - vertices[i_idx[0]], 
                                        vertices[i_idx[2]] - vertices[i_idx[1]]));

        // Check all other triangles for shared pairs of vertices.
        for (size_t k = i + 1; k < num_triangles(); ++k) {
            // Pointer to first element of the k-th triangle's 3 indices.
            auto k_idx = &indices[k * 3];
            // Normal vector of k-th triangle.
            auto k_normal = normalize(cross(vertices[k_idx[1]] - vertices[k_idx[0]], 
                                            vertices[k_idx[2]] - vertices[k_idx[1]]));

            // Check which vertices are shared.
            bool shared_idx[] = {false, false, false};

            for (size_t j = 0; j < 3; ++j) {
                shared_idx[j] = i_idx[j] == k_idx[0] || i_idx[j] == k_idx[1] || i_idx[j] == k_idx[2];
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
                    auto sign = dot(i_normal, vertices[other_idx] - vertices[i_idx[m]]);
                    auto concave = sign > 0;
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

}