#include "edyn/shapes/triangle_mesh.hpp"

namespace edyn {

void triangle_mesh::initialize() {
    build_tree();
    initialize_edge_angles();
    calculate_edge_angles();
}

void triangle_mesh::initialize_edge_angles() {
    cos_angles.resize(indices.size());
    is_concave_edge.resize(indices.size());

    std::fill(cos_angles.begin(), cos_angles.end(), scalar(-1));
    std::fill(is_concave_edge.begin(), is_concave_edge.end(), false);
}

void triangle_mesh::calculate_edge_angles() {
    for (size_t i = 0; i < num_triangles(); ++i) {
        // Pointer to first element of the i-th triangle's 3 indices.
        auto i_idx = &indices[i * 3];
        // Triangle vertices.
        auto i_vertex = triangle_vertices{
            vertices[i_idx[0]],
            vertices[i_idx[1]],
            vertices[i_idx[2]]
        };
        // Normal vector of i-th triangle.
        auto i_edge0 = i_vertex[1] - i_vertex[0];
        auto i_edge1 = i_vertex[2] - i_vertex[1];
        auto i_normal = cross(i_edge0, i_edge1);
        auto i_normal_len_sqr = length_sqr(i_normal);

        if (i_normal_len_sqr > EDYN_EPSILON) {
            i_normal /= std::sqrt(i_normal_len_sqr);
        }

        // Find shared pairs of vertices with other triangles.        
        auto tri_aabb = get_triangle_aabb(i_vertex);
        auto inset = vector3 {-EDYN_EPSILON, -EDYN_EPSILON, -EDYN_EPSILON};
        tree.visit(tri_aabb.inset(inset), [&] (auto k) {
            if (k == i) {
                return;
            }

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
                return;
            }

            // Look for pairs of shared indices which translates to a shared edge.
            for (size_t m = 0; m < 3; ++m) {
                auto next_m = (m + 1) % 3;
                if (shared_idx[m] && shared_idx[next_m]) {
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

                    // Normal vector of k-th triangle.
                    auto k_edge0 = vertices[k_idx[1]] - vertices[k_idx[0]];
                    auto k_edge1 = vertices[k_idx[2]] - vertices[k_idx[1]];
                    auto k_normal = cross(k_edge0, k_edge1);
                    auto k_normal_len_sqr = length_sqr(k_normal);

                    if (k_normal_len_sqr > EDYN_EPSILON) {
                        k_normal /= std::sqrt(k_normal_len_sqr);
                    }

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

                    cos_angles[k * 3 + n] = cos_angle;
                    is_concave_edge[k * 3 + n] = concave;

                    // There can be only one shared edge.
                    break;
                }
            }
        });
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

        auto tri_aabb = get_triangle_aabb(verts);
        aabbs.push_back(tri_aabb);
    }

    auto report_leaf = [] (static_tree::tree_node &node, auto ids_begin, auto ids_end) {
        node.id = *ids_begin;
    };
    tree.build(aabbs.begin(), aabbs.end(), report_leaf);
}

}