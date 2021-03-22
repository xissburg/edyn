#ifndef EDYN_SHAPES_CONVEX_MESH_HPP
#define EDYN_SHAPES_CONVEX_MESH_HPP

#include <array>
#include <vector>
#include <cstdint>
#include "edyn/math/vector3.hpp"
#include "edyn/config/config.h"

namespace edyn {

struct convex_mesh {
    std::vector<vector3> vertices;
    std::vector<uint16_t> edges;
    std::vector<uint16_t> indices;
    std::vector<vector3> normals;

    size_t num_triangles() const {
        EDYN_ASSERT(indices.size() % 3 == 0);
        return indices.size() / 3;
    }

    std::array<vector3, 3> get_triangle(size_t i) const {
        EDYN_ASSERT(i * 3 + 2 < indices.size());
        return {
            vertices[indices[i * 3 + 0]],
            vertices[indices[i * 3 + 1]],
            vertices[indices[i * 3 + 2]],
        };
    }
    
    void calculate_normals() {
        for (size_t i = 0; i < num_triangles(); ++i) {
            auto vertices = get_triangle(i);
            auto normal = cross(vertices[1] - vertices[0], vertices[2] - vertices[1]);
            normal = normalize(normal);
            normals.push_back(normal);
        }
    }
};

}

#endif // EDYN_SHAPES_CONVEX_MESH_HPP
