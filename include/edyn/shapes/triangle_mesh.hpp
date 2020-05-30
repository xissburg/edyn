#ifndef EDYN_SHAPES_TRIANGLE_MESH_HPP
#define EDYN_SHAPES_TRIANGLE_MESH_HPP

#include <cstdint>
#include <vector>
#include "edyn/math/vector3.hpp"
#include "edyn/math/geom.hpp"
#include "edyn/comp/aabb.hpp"
#include "edyn/shapes/triangle_shape.hpp"
#include "edyn/collision/static_tree.hpp"

namespace edyn {

struct triangle_mesh {
    std::vector<vector3> vertices;
    std::vector<uint16_t> indices;
    std::vector<scalar> cos_angles;
    std::vector<bool> is_concave_edge;
    AABB aabb;
    static_tree tree;

    size_t num_triangles() const {
        EDYN_ASSERT(indices.size() % 3 == 0);
        return indices.size() / 3;
    }

    template<typename Func>
    void visit(const AABB &aabb, Func func) const {
        constexpr auto inset = vector3 {
            -contact_breaking_threshold, 
            -contact_breaking_threshold, 
            -contact_breaking_threshold
        };
        
        tree.visit(aabb.inset(inset), [&] (auto tri_idx) {
            auto verts = triangle_vertices{
                vertices[indices[tri_idx * 3 + 0]],
                vertices[indices[tri_idx * 3 + 1]],
                vertices[indices[tri_idx * 3 + 2]]
            };

            func(tri_idx, verts);
        });
    }

    void initialize();
    void calculate_aabb();
    void calculate_edge_angles();
    void build_tree();
};

}

#endif // EDYN_SHAPES_TRIANGLE_MESH_HPP