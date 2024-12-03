#include "edyn/serialization/paged_triangle_mesh_s11n.hpp"
#include "edyn/context/task.hpp"
#include "edyn/serialization/triangle_mesh_s11n.hpp"
#include "edyn/serialization/static_tree_s11n.hpp"
#include "edyn/serialization/math_s11n.hpp"
#include "edyn/serialization/std_s11n.hpp"
#include "edyn/serialization/memory_archive.hpp"
#include "edyn/shapes/paged_triangle_mesh.hpp"
#include "edyn/shapes/triangle_mesh.hpp"
#include <entt/signal/delegate.hpp>
#include <cstdint>
#include <memory>
#include <string>

namespace edyn {

std::string get_submesh_path(const std::string &paged_triangle_mesh_path, size_t index) {
    auto submesh_path = paged_triangle_mesh_path;
    auto dot_pos = submesh_path.rfind('.');

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

void serialize(paged_triangle_mesh_file_output_archive &archive,
               paged_triangle_mesh &paged_tri_mesh) {

    archive.m_triangle_mesh_index = 0;

    archive(paged_tri_mesh.m_thickness);
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
    archive(paged_tri_mesh.m_thickness);
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
    }

    // Resize LRU queue to have the number of submeshes.
    paged_tri_mesh.m_lru_indices.resize(num_submeshes);
    std::iota(paged_tri_mesh.m_lru_indices.begin(),
              paged_tri_mesh.m_lru_indices.end(), 0);

    paged_tri_mesh.m_is_loading_submesh = std::make_unique<std::atomic<bool>[]>(num_submeshes);
}


void paged_triangle_mesh_file_input_archive::load(paged_triangle_mesh *trimesh, size_t index) {
    auto load_page = load_mesh_context();
    load_page.m_input = reinterpret_cast<intptr_t>(this);
    load_page.m_trimesh = reinterpret_cast<intptr_t>(trimesh);
    load_page.m_index = index;

    if (m_enqueue_task) {
        auto *load_page_ptr = new load_mesh_context(std::move(load_page));
        auto task = task_delegate_t(entt::connect_arg_t<&load_mesh_context::load>{}, *load_page_ptr);
        auto completion = task_completion_delegate_t(entt::connect_arg_t<&load_mesh_context::completion>{}, *load_page_ptr);
        (*m_enqueue_task)(task, 1, completion);
    } else {
        load_page.load(0, 0);
    }
}

void load_mesh_context::load(unsigned start, unsigned end) {
    auto *input = reinterpret_cast<paged_triangle_mesh_file_input_archive *>(m_input);
    auto mesh = std::make_shared<triangle_mesh>();

    switch(input->m_mode) {
    case paged_triangle_mesh_serialization_mode::embedded:
        input->seek_position(input->m_base_offset + input->m_offsets[m_index]);
        serialize(*input, *mesh);
        break;
    case paged_triangle_mesh_serialization_mode::external: {
        auto tri_mesh_path = get_submesh_path(input->m_path, m_index);
        auto tri_mesh_archive = file_input_archive(tri_mesh_path);
        serialize(tri_mesh_archive, *mesh);
        break;
    }
    }

    auto *paged = reinterpret_cast<paged_triangle_mesh *>(m_trimesh);
    paged->assign_mesh(m_index, mesh);
}

void load_mesh_context::completion() {
    delete this;
}

}
