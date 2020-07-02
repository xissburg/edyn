#ifndef EDYN_SHAPES_TRIANGLE_MESH_PAGE_LOADER_HPP
#define EDYN_SHAPES_TRIANGLE_MESH_PAGE_LOADER_HPP

namespace edyn {

class triangle_mesh;

class triangle_mesh_page_loader_base {
public:
    virtual void load(size_t i, triangle_mesh &mesh) = 0;
};


}

#endif // EDYN_SHAPES_TRIANGLE_MESH_PAGE_LOADER_HPP