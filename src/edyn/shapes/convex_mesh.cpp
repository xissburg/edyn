#include "edyn/shapes/convex_mesh.hpp"
#include "edyn/config/config.h"
#include "edyn/sys/update_rotated_meshes.hpp"
#include "edyn/util/shape_util.hpp"
#include "edyn/config/constants.hpp"
#include <limits>

namespace edyn {

void convex_mesh::initialize() {
    shift_to_centroid();
    update_calculated_properties();
    EDYN_ASSERT(validate());
}

void convex_mesh::update_calculated_properties() {
    calculate_normals();
    calculate_edges();
    calculate_neighbors();
    calculate_relevant_faces();
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

vector3 convex_mesh::get_edge_direction(size_t idx) const {
    auto vertices = get_edge(idx);
    return vertices[1] - vertices[0];
}

vector3 convex_mesh::get_rotated_edge_direction(const rotated_mesh &rmesh,
                                                size_t idx) const {
    auto vertices = get_rotated_edge(rmesh, idx);
    return vertices[1] - vertices[0];
}

std::array<uint32_t, 2> convex_mesh::get_edge_vertices(size_t idx) const {
    EDYN_ASSERT(idx * 2 + 1 < edges.size());
    return {
        edges[idx * 2],
        edges[idx * 2 + 1]
    };
}

std::array<uint32_t, 2> convex_mesh::get_edge_faces(size_t idx) const {
    EDYN_ASSERT(idx * 2 + 1 < edges.size());
    return {
        edge_faces[idx * 2],
        edge_faces[idx * 2 + 1]
    };
}

std::array<vector3, 2> convex_mesh::get_edge_face_normals(size_t idx) const {
    EDYN_ASSERT(idx * 2 + 1 < edges.size());
    return {
        normals[edge_faces[idx * 2]],
        normals[edge_faces[idx * 2 + 1]],
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
        auto normal = vector3_zero;

        // Find a second edge that's not collinear.
        for (size_t j = 1; j < count; ++j) {
            auto i2 = indices[first + j];
            auto i3 = indices[first + (j + 1) % count];
            auto &v2 = vertices[i2];
            auto &v3 = vertices[i3];
            auto n = cross(v1 - v0, v3 - v2);

            if (try_normalize(n)) {
                normal = n;
                break;
            }
        }

        if (normal == vector3_zero) {
            normal = vector3_y;
        }

        normals.push_back(normal);
    }

    EDYN_ASSERT(normals.size() == num_faces());
}

void convex_mesh::calculate_edges() {
    edges.clear();

    for (size_t face_idx = 0; face_idx < num_faces(); ++face_idx) {
        const auto first = faces[face_idx * 2];
        const auto count = faces[face_idx * 2 + 1];

        for (size_t face_vertex_idx = 0; face_vertex_idx < count; ++face_vertex_idx) {
            auto vertex_idx0 = indices[first + face_vertex_idx];
            auto vertex_idx1 = indices[first + (face_vertex_idx + 1) % count];
            auto contains = false;

            for (size_t edge_idx = 0; edge_idx < num_edges(); ++edge_idx) {
                auto edge_vertex_idx = get_edge_vertices(edge_idx);

                if ((edge_vertex_idx[0] == vertex_idx0 && edge_vertex_idx[1] == vertex_idx1) ||
                    (edge_vertex_idx[0] == vertex_idx1 && edge_vertex_idx[1] == vertex_idx0)) {
                    contains = true;

                    // Assign second incident face index to known edge.
                    // The cross product of incident face normals point in the same
                    // direction as the edge, because due to convexity and ccw
                    // winding order in all faces, the second normal will always
                    // point outside of the face that was processed first and the
                    // edge will be directed along the circumference of the first face
                    // as well.
                    EDYN_ASSERT(edge_faces[edge_idx * 2 + 1] == std::numeric_limits<uint32_t>::max());
                    edge_faces[edge_idx * 2 + 1] = face_idx;
                    break;
                }
            }

            if (!contains) {
                edges.push_back(vertex_idx0);
                edges.push_back(vertex_idx1);
                edge_faces.push_back(face_idx);
                edge_faces.push_back(std::numeric_limits<uint32_t>::max());
            }
        }
    }
}

void convex_mesh::calculate_neighbors() {
    neighbors_start.push_back(0);
    uint32_t neighbor_count = 0;

    for (size_t vertex_idx = 0; vertex_idx < vertices.size(); ++vertex_idx) {
        // Find all edges that contain this vertex and add the other vertex to
        // the list of neighbors.
        for (size_t edge_idx = 0; edge_idx < num_edges(); ++edge_idx) {
            auto edge_vertices = get_edge_vertices(edge_idx);

            if (edge_vertices[0] == vertex_idx || edge_vertices[1] == vertex_idx) {
                auto neighbor_idx = edge_vertices[0] == vertex_idx ? edge_vertices[1] : edge_vertices[0];
                neighbor_indices.push_back(neighbor_idx);
                ++neighbor_count;
            }
        }

        neighbors_start.push_back(neighbor_count);
    }
}

void convex_mesh::calculate_relevant_faces() {
    // Find unique face normals.
    for (size_t face_idx = 0; face_idx < num_faces(); ++face_idx) {
        auto &normal = normals[face_idx];
        auto found_it = std::find_if(relevant_faces.begin(), relevant_faces.end(), [&](auto other_face_idx) {
            auto other_normal = normals[other_face_idx];
            return !(dot(normal, other_normal) < scalar(1) - convex_mesh_relevant_direction_tolerance);
        });

        if (found_it == relevant_faces.end()) {
            relevant_faces.push_back(face_idx);
        }
    }
}

void convex_mesh::calculate_relevant_edges() {
    // Find unique edge directions. Parallel edges that point in opposite
    // directions are considered similar since the direction doesn't matter
    // when doing edge vs edge in SAT.
    for (size_t edge_idx = 0; edge_idx < num_edges(); ++edge_idx) {
        auto edge = normalize(get_edge_direction(edge_idx));

        auto found_it = std::find_if(relevant_edges.begin(), relevant_edges.end(), [this, edge](auto other_edge_idx) {
            auto other_edge = normalize(get_edge_direction(other_edge_idx));
            return !(std::abs(dot(edge, other_edge)) < scalar(1) - convex_mesh_relevant_direction_tolerance);
        });

        if (found_it == relevant_edges.end()) {
            relevant_edges.push_back(edge_idx);
        }
    }
}

bool convex_mesh::validate() const {
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

            if (std::abs(dot(vj - v0, normal)) > convex_mesh_validation_parallel_tolerance) {
                return false;
            }
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
            if(dot(vj - v0, normal) > convex_mesh_validation_parallel_tolerance) {
                return false;
            }
        }
    }

    return true;
}

rotated_mesh make_rotated_mesh(const convex_mesh &mesh, const quaternion &orn) {
    auto rotated = rotated_mesh{};
    rotated.vertices.resize(mesh.vertices.size());
    rotated.normals.resize(mesh.normals.size());

    update_rotated_mesh(rotated, mesh, orn);

    return rotated;
}

}
