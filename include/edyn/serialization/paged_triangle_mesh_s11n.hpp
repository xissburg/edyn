#ifndef EDYN_SERIALIZATION_PAGED_TRIANGLE_MESH_S11N_HPP
#define EDYN_SERIALIZATION_PAGED_TRIANGLE_MESH_S11N_HPP

#include "edyn/shapes/paged_triangle_mesh.hpp"
#include "edyn/shapes/triangle_mesh_page_loader.hpp"
#include "edyn/serialization/memory_archive.hpp"
#include "edyn/serialization/file_archive.hpp"

namespace edyn {

class paged_triangle_mesh_memory_output_archive: public memory_output_archive {
public:
    using super = memory_output_archive;

    paged_triangle_mesh_memory_output_archive(super::buffer_type& buffer)
        : super(buffer)
    {}

    void operator()(paged_triangle_mesh &paged_tri_mesh) {
        super::operator()(paged_tri_mesh.m_tree);
        super::operator()(paged_tri_mesh.m_cache.size());
        size_t tri_mesh_offset = 0;

        for (auto &entry : paged_tri_mesh.m_cache) {
            super::operator()(entry.num_vertices);
            super::operator()(entry.num_indices);

            super::operator()(tri_mesh_offset);
            auto tri_mesh_size = serialization_sizeof(*entry.trimesh);
            tri_mesh_offset += tri_mesh_size;
        }

        for (auto &entry : paged_tri_mesh.m_cache) {
            super::operator()(*entry.trimesh);
        }
    }
};

class paged_triangle_mesh_memory_input_archive: public memory_input_archive, public triangle_mesh_page_loader_base {
public:
    using super = memory_input_archive;

    paged_triangle_mesh_memory_input_archive(super::buffer_type& buffer)
        : super(buffer)
    {}

    void operator()(paged_triangle_mesh &paged_tri_mesh) {
        super::operator()(paged_tri_mesh.m_tree);

        size_t size;
        memory_input_archive::operator()(size);
        paged_tri_mesh.m_cache.resize(size);
        m_offsets.resize(size);

        for (size_t i = 0; i < size; ++i) {
            auto &entry = paged_tri_mesh.m_cache[i];
            memory_input_archive::operator()(entry.num_vertices);
            memory_input_archive::operator()(entry.num_indices);
            memory_input_archive::operator()(m_offsets[i]);
        }

        m_base_offset = m_position;
    }

    void load(size_t i, triangle_mesh &mesh) override {
        m_position = m_offsets[i];
        memory_input_archive::operator()(mesh);
    }

private:
    size_t m_base_offset;
    std::vector<size_t> m_offsets;
};

}

#endif // EDYN_SERIALIZATION_PAGED_TRIANGLE_MESH_S11N_HPP