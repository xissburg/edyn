#ifndef EDYN_SERIALIZATION_PAGED_TRIANGLE_MESH_S11N_HPP
#define EDYN_SERIALIZATION_PAGED_TRIANGLE_MESH_S11N_HPP

#include <type_traits>
#include "edyn/shapes/paged_triangle_mesh.hpp"
#include "edyn/shapes/triangle_mesh_page_loader.hpp"
#include "edyn/serialization/file_archive.hpp"
#include "edyn/parallel/job_queue_scheduler.hpp"
#include "edyn/parallel/job.hpp"
#include <entt/signal/sigh.hpp>

namespace edyn {

class paged_triangle_mesh_file_output_archive;
class paged_triangle_mesh_file_input_archive;
class load_mesh_job;
class finish_load_mesh_job;

void serialize(paged_triangle_mesh_file_output_archive &archive,
               paged_triangle_mesh &paged_tri_mesh);

void serialize(paged_triangle_mesh_file_input_archive &archive,
               paged_triangle_mesh &paged_tri_mesh);

/**
 * A `paged_triangle_mesh` can have each of its submeshes serialized into separate
 * files or have everything inside a single file.
 */
enum class paged_triangle_mesh_serialization_mode: uint8_t {
    /**
     * Embeds submeshes inside the same file as the `paged_triangle_mesh` and
     * uses offsets within this file to load submeshes on demand.
     */
    embedded,

    /**
     * Writes to/reads from separate individual files for each submesh as needed.
     */
    external
};

template<typename Archive>
void serialize(Archive &archive, paged_triangle_mesh_serialization_mode &mode) {
    archive(*reinterpret_cast<std::underlying_type<paged_triangle_mesh_serialization_mode>::type*>(&mode));
}

/**
 * Get the filename of a submesh for a `paged_triangle_mesh` created with
 * `external` serialization mode.
 * @param paged_triangle_mesh_path Path of the `paged_triangle_mesh`.
 * @param index Index of submesh.
 * @return Path of submesh file which can be loaded into a `triangle_mesh`
 *      instance.
 */
std::string get_submesh_path(const std::string &paged_triangle_mesh_path, size_t index);

/**
 * Specialized archive to write a `paged_triangle_mesh` to file.
 */
class paged_triangle_mesh_file_output_archive: public file_output_archive {
public:
    using super = file_output_archive;

    paged_triangle_mesh_file_output_archive(const std::string &path,
                                            paged_triangle_mesh_serialization_mode mode)
        : super(path)
        , m_path(path)
        , m_triangle_mesh_index(0)
        , m_mode(mode)
    {}

    template<typename... Ts>
    void operator()(Ts&... t) {
        super::operator()(t...);
    }

    template<typename T>
    void operator()(T& t) {
        super::operator()(t);
    }

    void operator()(triangle_mesh &tri_mesh);

    auto get_serialization_mode() const {
        return m_mode;
    }

    friend void serialize(paged_triangle_mesh_file_output_archive &archive,
                          paged_triangle_mesh &paged_tri_mesh);

private:
    std::string m_path;
    size_t m_triangle_mesh_index;
    paged_triangle_mesh_serialization_mode m_mode;
};

/**
 * Specialized archive to read a `paged_triangle_mesh` from file. It can also be
 * used as a page loader in the `paged_triangle_mesh`.
 */
class paged_triangle_mesh_file_input_archive: public file_input_archive, public triangle_mesh_page_loader_base {
public:
    using super = file_input_archive;

    paged_triangle_mesh_file_input_archive() {}

    paged_triangle_mesh_file_input_archive(const std::string &path)
        : super(path)
        , m_path(path)
    {}

    void open(const std::string &path) {
        super::open(path);
        m_path = path;
    }

    void load(size_t index) override;

    virtual entt::sink<entt::sigh<loaded_mesh_func_t>> on_load_sink() override {
        return {m_loaded_signal};
    }

    friend void serialize(paged_triangle_mesh_file_input_archive &archive,
                          paged_triangle_mesh &paged_tri_mesh);
    friend void load_mesh_job_func(job::data_type &);
    friend void finish_load_mesh_job_func(job::data_type &);

private:
    std::string m_path;
    size_t m_base_offset;
    std::vector<size_t> m_offsets;
    paged_triangle_mesh_serialization_mode m_mode;
    entt::sigh<loaded_mesh_func_t> m_loaded_signal;
};

/**
 * Job used to load submeshes in the background.
 */
struct load_mesh_context {
    // Integral value of a pointer to and instance of
    // `paged_triangle_mesh_file_input_archive`.
    intptr_t m_input;
    // Index of submesh to be loaded.
    size_t m_index;
};

void load_mesh_job_func(job::data_type &);

}

#endif // EDYN_SERIALIZATION_PAGED_TRIANGLE_MESH_S11N_HPP
