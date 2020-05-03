#ifndef EDYN_SHAPES_TRIANGLE_MESH_HPP
#define EDYN_SHAPES_TRIANGLE_MESH_HPP

#include <cstdint>
#include <vector>
#include "edyn/math/vector3.hpp"
#include "edyn/math/geom.hpp"
#include "edyn/comp/aabb.hpp"
#include "edyn/shapes/triangle_shape.hpp"

namespace edyn {

struct triangle_mesh {
    std::vector<vector3> vertices;
    std::vector<uint16_t> indices;
    std::vector<uint16_t> adjacency;
    std::vector<scalar> cos_angles;
    std::vector<bool> is_concave_edge;
    AABB aabb;

    size_t num_triangles() const {
        EDYN_ASSERT(indices.size() % 3 == 0);
        return indices.size() / 3;
    }

    template<typename Func>
    void visit(const AABB &aabb, Func func) const {
        constexpr auto offset = vector3 {
            contact_breaking_threshold, 
            contact_breaking_threshold, 
            contact_breaking_threshold
        };
        
        // TODO: use bounding volume hierarchy tree.
        for (size_t i = 0; i < num_triangles(); ++i) {
            auto verts = triangle_vertices{
                vertices[indices[i * 3 + 0]],
                vertices[indices[i * 3 + 1]],
                vertices[indices[i * 3 + 2]]
            };
            auto tri_min = min(min(verts[0], verts[1]), verts[2]);
            auto tri_max = max(max(verts[0], verts[1]), verts[2]);

            if (intersect_aabb(aabb.min - offset, aabb.max + offset, 
                               tri_min - offset, tri_max + offset)) {
                func(i, verts);
            }
        }
    }

    void calculate_aabb();
    void calculate_adjacency();
};

}

#endif // EDYN_SHAPES_TRIANGLE_MESH_HPP