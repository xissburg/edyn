#include "edyn/serialization/paged_triangle_mesh_s11n.hpp"
#include "edyn/serialization/triangle_mesh_s11n.hpp"
#include "edyn/serialization/static_tree_s11n.hpp"
#include "edyn/serialization/math_s11n.hpp"
#include "edyn/serialization/std_s11n.hpp"

namespace edyn {

std::string get_submesh_path(const std::string &paged_triangle_mesh_path, size_t index) {
    auto submesh_path = paged_triangle_mesh_path;
    auto dot_pos = submesh_path.find(".");

    if (dot_pos != std::string::npos) {
        submesh_path.insert(dot_pos, std::to_string(index));
    } else {
        submesh_path += std::to_string(index);
    }

    return submesh_path;
}

void paged_triangle_mesh_file_output_archive::operator()(triangle_mesh &tri_mesh) {
    switch(m_mode) {
    case paged_triangle_mesh_serialization_mode::embedded:
        serialize(*this, tri_mesh);
        break;
    case paged_triangle_mesh_serialization_mode::external: {
        auto tri_mesh_path = get_submesh_path(m_path, m_triangle_mesh_index);
        auto archive = file_output_archive(tri_mesh_path);
        serialize(archive, tri_mesh);
        break;
    }
    }
    ++m_triangle_mesh_index;
}

void paged_triangle_mesh_file_input_archive::load(size_t index, triangle_mesh &mesh) {
    switch(m_mode) {
    case paged_triangle_mesh_serialization_mode::embedded:
        seek_position(m_base_offset + m_offsets[index]);
        serialize(*this, mesh);
        break;
    case paged_triangle_mesh_serialization_mode::external: {
        auto tri_mesh_path = get_submesh_path(m_path, index);
        auto archive = file_input_archive(tri_mesh_path);
        serialize(archive, mesh);
        break;
    }
    }
}

void serialize(paged_triangle_mesh_file_output_archive &archive, 
               paged_triangle_mesh &paged_tri_mesh) {
    archive(paged_tri_mesh.m_tree);
    auto num_submeshes = paged_tri_mesh.m_cache.size();
    archive(num_submeshes);

    for (auto &entry : paged_tri_mesh.m_cache) {
        archive(entry.num_vertices);
        archive(entry.num_indices);
    }

    archive(archive.m_mode);

    if (archive.m_mode == paged_triangle_mesh_serialization_mode::embedded) {
        size_t tri_mesh_offset = 0;
        for (size_t i = 0; i < num_submeshes; ++i) {
            archive(tri_mesh_offset);
            auto tri_mesh_size = serialization_sizeof(*paged_tri_mesh.m_cache[i].trimesh);
            tri_mesh_offset += tri_mesh_size;
        }
    }

    for (auto &entry : paged_tri_mesh.m_cache) {
        archive(*entry.trimesh);
    }
}

void serialize(paged_triangle_mesh_file_input_archive &archive, 
               paged_triangle_mesh &paged_tri_mesh) {
    archive(paged_tri_mesh.m_tree);

    size_t num_submeshes;
    archive(num_submeshes);
    paged_tri_mesh.m_cache.resize(num_submeshes);

    for (size_t i = 0; i < num_submeshes; ++i) {
        auto &entry = paged_tri_mesh.m_cache[i];
        archive(entry.num_vertices);
        archive(entry.num_indices);
    }

    archive(archive.m_mode);

    if (archive.m_mode == paged_triangle_mesh_serialization_mode::embedded) {
        archive.m_offsets.resize(num_submeshes);

        for (size_t i = 0; i < num_submeshes; ++i) {
            archive(archive.m_offsets[i]);
        }

        archive.m_base_offset = archive.tell_position();
        archive.m_mode = paged_triangle_mesh_serialization_mode::embedded;
    }
}

}