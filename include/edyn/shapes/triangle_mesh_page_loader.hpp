#ifndef EDYN_SHAPES_TRIANGLE_MESH_PAGE_LOADER_HPP
#define EDYN_SHAPES_TRIANGLE_MESH_PAGE_LOADER_HPP

namespace edyn {

class triangle_mesh;

class triangle_mesh_page_loader_base {
public:
    virtual void load(size_t i, triangle_mesh &mesh) = 0;
};

/**
 * @brief Triangle mesh loader with an input archive source.
 */
template<typename InputArchiveSource>
class triangle_mesh_page_loader: public triangle_mesh_page_loader_base {
public:
    triangle_mesh_page_loader(const InputArchiveSource& source)
        : m_source(source)
    {}

    void load(size_t i, triangle_mesh &mesh) override {
        auto input = m_source(i);
        serialize(input, mesh);
    }

private:
    InputArchiveSource m_source;
};

}

#endif // EDYN_SHAPES_TRIANGLE_MESH_PAGE_LOADER_HPP