#include "edyn/shapes/convex_mesh.hpp"
#include "edyn/math/scalar.hpp"
#include "edyn/math/vector3.hpp"
#include "edyn/sys/update_rotated_meshes.hpp"

namespace edyn {

void convex_mesh::initialize() {
    calculate_normals();
    calculate_edges();

#ifdef EDYN_DEBUG
    validate();
#endif
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

        // Find a second edge that's not colinear.
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

void convex_mesh::validate() const {
    // Check if all faces are flat.
    for (size_t i = 0; i < num_faces(); ++i) {
        auto &normal = normals[i];
        auto first = faces[i * 2];
        auto count = faces[i * 2 + 1];
        auto i0 = indices[first];
        auto &v0 = vertices[i0];

        // Find a second edge that's not colinear.
        for (size_t j = 1; j < count; ++j) {
            auto ij = indices[first + j];
            auto &vj = vertices[ij];

            EDYN_ASSERT(std::abs(dot(vj - v0, normal)) < EDYN_EPSILON);
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
            EDYN_ASSERT(dot(vj - v0, normal) < EDYN_EPSILON);
        }
    }
}

rotated_mesh make_rotated_mesh(const convex_mesh &mesh, const quaternion &orn) {
    auto rotated = rotated_mesh{};
    rotated.vertices.resize(mesh.vertices.size());
    rotated.normals.resize(mesh.normals.size());

    update_rotated_mesh(rotated, mesh, orn);

    return rotated;
}

}
