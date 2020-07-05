#ifndef EDYN_SHAPES_TRIANGLE_MESH_PAGE_LOADER_HPP
#define EDYN_SHAPES_TRIANGLE_MESH_PAGE_LOADER_HPP

#include <memory>
#include <entt/signal/fwd.hpp>

namespace edyn {

class triangle_mesh;

class triangle_mesh_page_loader_base {
public:
    using loaded_mesh_func_t = void(size_t, std::unique_ptr<triangle_mesh> &);
    virtual void load(size_t index) = 0;
    virtual entt::sink<loaded_mesh_func_t> loaded_mesh_sink() = 0;
};


}

#endif // EDYN_SHAPES_TRIANGLE_MESH_PAGE_LOADER_HPP