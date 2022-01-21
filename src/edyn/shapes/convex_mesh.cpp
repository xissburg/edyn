#include "edyn/shapes/convex_mesh.hpp"
#include "edyn/sys/update_rotated_meshes.hpp"
#include "edyn/util/shape_util.hpp"

namespace edyn {

void convex_mesh::initialize() {
    shift_to_centroid();
    update_calculated_properties();

#ifdef EDYN_DEBUG
    validate();
#endif
}

void convex_mesh::update_calculated_properties() {
    calculate_normals();
    calculate_edges();
    calculate_relevant_normals();
    calculate_relevant_edges();
}

void convex_mesh::shift_to_centroid() {
    auto center = mesh_centroid(vertices, indices, faces);

    for (auto &v : vertices) {
        v -= center;
    }
}

std::array<vector3, 2> convex_mesh::get_edge(size_t idx) const {
    EDYN_ASSERT(idx * 2 + 1 < edges.size());
    return {
        vertices[edges[idx * 2 + 0]],
        vertices[edges[idx * 2 + 1]]
    };
}

std::array<vector3, 2> convex_mesh::get_rotated_edge(const rotated_mesh &rmesh,
                                                     size_t idx) const {
    EDYN_ASSERT(idx * 2 + 1 < edges.size());
    EDYN_ASSERT(rmesh.vertices.size() == vertices.size());
    return {
        rmesh.vertices[edges[idx * 2 + 0]],
        rmesh.vertices[edges[idx * 2 + 1]]
    };
}

void convex_mesh::calculate_normals() {
    normals.clear();

    for (size_t i = 0; i < num_faces(); ++i) {
        auto first = faces[i * 2];
        auto count = faces[i * 2 + 1];
        // Grab first edge.
        auto i0 = indices[first];
        auto i1 = indices[first + 1];
        auto &v0 = vertices[i0];
        auto &v1 = vertices[i1];

        // Find a second edge that's not collinear.
        for (size_t j = 1; j < count; ++j) {
            auto i2 = indices[first + j];
            auto i3 = indices[first + (j + 1) % count];
            auto &v2 = vertices[i2];
            auto &v3 = vertices[i3];

            auto normal = cross(v1 - v0, v3 - v2);
            auto normal_len_sqr = length_sqr(normal);

            if (normal_len_sqr > EDYN_EPSILON) {
                normal /= std::sqrt(normal_len_sqr);
                normals.push_back(normal);
                break;
            }
        }
    }

    EDYN_ASSERT(normals.size() == num_faces());
}

void convex_mesh::calculate_edges() {
    edges.clear();

    for (size_t i = 0; i < num_faces(); ++i) {
        const auto first = faces[i * 2];
        const auto count = faces[i * 2 + 1];

        for (size_t j = 0; j < count; ++j) {
            auto contains = false;
            auto i0 = indices[first + j];
            auto i1 = indices[first + (j + 1) % count];

            for (size_t k = 0; k < edges.size(); k += 2) {
                if ((edges[k] == i0 && edges[k + 1] == i1) ||
                    (edges[k] == i1 && edges[k + 1] == i0)) {
                    contains = true;
                    break;
                }
            }

            if (!contains) {
                edges.push_back(i0);
                edges.push_back(i1);
            }
        }
    }
}

void convex_mesh::calculate_relevant_normals() {
    // Find unique face normals.
    for (size_t face_idx = 0; face_idx < normals.size(); ++face_idx) {
        auto &normal = normals[face_idx];
        auto found_it = std::find_if(relevant_normals.begin(), relevant_normals.end(), [normal] (auto &&relevant) {
            return !(dot(normal, relevant) < scalar(1) - EDYN_EPSILON);
        });

        if (found_it == relevant_normals.end()) {
            relevant_normals.push_back(normal);
            auto v_idx = first_vertex_index(face_idx);
            relevant_indices.push_back(v_idx);
        }
    }
}

void convex_mesh::calculate_relevant_edges() {
    // Find unique edge directions. Parallel edges that point in opposite
    // directions are considered similar since the direction doesn't matter
    // when doing edge vs edge in SAT.
    for (size_t i = 0; i < edges.size(); i += 2) {
        auto i0 = edges[i];
        auto i1 = edges[i + 1];
        auto v0 = vertices[i0];
        auto v1 = vertices[i1];
        auto edge = normalize(v1 - v0);

        auto found_it = std::find_if(relevant_edges.begin(), relevant_edges.end(), [edge] (auto &&relevant) {
            return !(std::abs(dot(edge, relevant)) < scalar(1) - EDYN_EPSILON);
        });

        if (found_it == relevant_edges.end()) {
            relevant_edges.push_back(edge);
        }
    }
}

#ifdef EDYN_DEBUG
void convex_mesh::validate() const {
#ifndef EDYN_DISABLE_ASSERT
    // Check if all faces are flat.
    for (size_t i = 0; i < num_faces(); ++i) {
        auto &normal = normals[i];
        auto first = faces[i * 2];
        auto count = faces[i * 2 + 1];
        auto i0 = indices[first];
        auto &v0 = vertices[i0];

        // Find a second edge that's not collinear.
        for (size_t j = 1; j < count; ++j) {
            auto ij = indices[first + j];
            auto &vj = vertices[ij];

            EDYN_ASSERT(std::abs(dot(vj - v0, normal)) < edyn::scalar(0.001));
        }
    }

    // Check if mesh is convex.
    for (size_t i = 0; i < num_faces(); ++i) {
        auto &normal = normals[i];
        auto first = faces[i * 2];
        auto i0 = indices[first];
        auto &v0 = vertices[i0];

        // All vertices must lie behind the plane parallel to the face.
        for (auto &vj : vertices) {
            EDYN_ASSERT(dot(vj - v0, normal) < edyn::scalar(0.001));
        }
    }
#endif
}
#endif

rotated_mesh make_rotated_mesh(const convex_mesh &mesh, const quaternion &orn) {
    auto rotated = rotated_mesh{};
    rotated.vertices.resize(mesh.vertices.size());
    rotated.relevant_normals.resize(mesh.relevant_normals.size());
    rotated.relevant_edges.resize(mesh.relevant_edges.size());

    update_rotated_mesh(rotated, mesh, orn);

    return rotated;
}

}
