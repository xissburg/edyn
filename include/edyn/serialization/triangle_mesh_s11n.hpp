#ifndef EDYN_SERIALIZATION_TRIANGLE_MESH_S11N_HPP
#define EDYN_SERIALIZATION_TRIANGLE_MESH_S11N_HPP

#include "edyn/shapes/triangle_mesh.hpp"
#include "edyn/serialization/std_s11n.hpp"
#include "edyn/serialization/static_tree_s11n.hpp"

namespace edyn {

template<typename Archive, typename T>
void serialize(Archive &archive, unordered_pair<T> &pair) {
    archive(pair.first);
    archive(pair.second);
}

template<typename T>
constexpr size_t serialization_sizeof(const unordered_pair<T> &pair) {
    return 2 * sizeof(T);
}

template<typename Archive, typename T>
void serialize(Archive &archive, flat_nested_array<T> &array) {
    archive(array.m_data);
    archive(array.m_range_starts);
}

template<typename T>
size_t serialization_sizeof(const flat_nested_array<T> &array) {
    return serialization_sizeof(array.m_data) + serialization_sizeof(array.m_range_starts);
}

template<typename Archive>
void serialize(Archive &archive, triangle_mesh &tri_mesh) {
    archive(tri_mesh.m_vertices);
    archive(tri_mesh.m_indices);
    archive(tri_mesh.m_normals);
    archive(tri_mesh.m_edge_vertex_indices);
    archive(tri_mesh.m_vertex_edge_indices);
    archive(tri_mesh.m_adjacent_normals);
    archive(tri_mesh.m_face_edge_indices);
    archive(tri_mesh.m_edge_face_indices);
    archive(tri_mesh.m_is_boundary_edge);
    archive(tri_mesh.m_is_convex_edge);
    archive(tri_mesh.m_triangle_tree);
    archive(tri_mesh.m_friction);
    archive(tri_mesh.m_restitution);
    archive(tri_mesh.m_material_ids);
    archive(tri_mesh.m_thickness);
}

inline
size_t serialization_sizeof(const triangle_mesh &tri_mesh) {
    return
        serialization_sizeof(tri_mesh.m_vertices) +
        serialization_sizeof(tri_mesh.m_indices) +
        serialization_sizeof(tri_mesh.m_normals) +
        serialization_sizeof(tri_mesh.m_edge_vertex_indices) +
        serialization_sizeof(tri_mesh.m_vertex_edge_indices) +
        serialization_sizeof(tri_mesh.m_adjacent_normals) +
        serialization_sizeof(tri_mesh.m_face_edge_indices) +
        serialization_sizeof(tri_mesh.m_edge_face_indices) +
        serialization_sizeof(tri_mesh.m_is_boundary_edge) +
        serialization_sizeof(tri_mesh.m_is_convex_edge) +
        serialization_sizeof(tri_mesh.m_triangle_tree) +
        serialization_sizeof(tri_mesh.m_friction) +
        serialization_sizeof(tri_mesh.m_restitution) +
        serialization_sizeof(tri_mesh.m_material_ids) +
        sizeof(tri_mesh.m_thickness);
}

}

#endif // EDYN_SERIALIZATION_TRIANGLE_MESH_S11N_HPP
