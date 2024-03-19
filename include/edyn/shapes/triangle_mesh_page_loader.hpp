#ifndef EDYN_SHAPES_TRIANGLE_MESH_PAGE_LOADER_HPP
#define EDYN_SHAPES_TRIANGLE_MESH_PAGE_LOADER_HPP

#include <cstddef>

namespace edyn {

class paged_triangle_mesh;

class triangle_mesh_page_loader_base {
public:
    virtual ~triangle_mesh_page_loader_base() = default;
    virtual void load(paged_triangle_mesh *trimesh, size_t index) = 0;
};


}

#endif // EDYN_SHAPES_TRIANGLE_MESH_PAGE_LOADER_HPP
