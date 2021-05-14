#ifndef EDYN_SERIALIZATION_TRIANGLE_MESH_S11N_HPP
#define EDYN_SERIALIZATION_TRIANGLE_MESH_S11N_HPP

#include "edyn/shapes/triangle_mesh.hpp"
#include "edyn/serialization/std_s11n.hpp"
#include "edyn/serialization/static_tree_s11n.hpp"

namespace edyn {

template<typename Archive>
void serialize(Archive &archive, triangle_mesh &tri_mesh) {
    archive(tri_mesh.m_vertices);
    archive(tri_mesh.m_indices);
    archive(tri_mesh.m_normals);
    archive(tri_mesh.m_edge_indices);
    archive(tri_mesh.m_edge_normals);
    archive(tri_mesh.m_vertex_tangents);
    archive(tri_mesh.m_vertex_tangent_ranges);
    archive(tri_mesh.m_face_edge_indices);
    archive(tri_mesh.m_edge_face_indices);
    archive(tri_mesh.m_boundary_edge_indices);
    archive(tri_mesh.m_is_concave_edge);
    archive(tri_mesh.m_is_concave_vertex);
    archive(tri_mesh.m_tree);
}

inline
size_t serialization_sizeof(const triangle_mesh &tri_mesh) {
    return
        serialization_sizeof(tri_mesh.m_vertices) +
        serialization_sizeof(tri_mesh.m_indices) +
        serialization_sizeof(tri_mesh.m_normals) +
        serialization_sizeof(tri_mesh.m_edge_indices) +
        serialization_sizeof(tri_mesh.m_edge_normals) +
        serialization_sizeof(tri_mesh.m_vertex_tangents) +
        serialization_sizeof(tri_mesh.m_vertex_tangent_ranges) +
        serialization_sizeof(tri_mesh.m_face_edge_indices) +
        serialization_sizeof(tri_mesh.m_edge_face_indices) +
        serialization_sizeof(tri_mesh.m_boundary_edge_indices) +
        serialization_sizeof(tri_mesh.m_is_concave_edge) +
        serialization_sizeof(tri_mesh.m_is_concave_vertex) +
        serialization_sizeof(tri_mesh.m_tree);
}

}

#endif // EDYN_SERIALIZATION_TRIANGLE_MESH_S11N_HPP
