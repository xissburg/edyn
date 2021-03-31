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

    // Vertex indices of all faces.
    std::vector<uint16_t> indices;

    // Each subsequent pair of integers represents the indices of the two 
    // vertices of an edge in the `vertices` array.
    std::vector<uint16_t> edges;

    // Each subsequent pair of integers represents the index of the first
    // vertex of a face in the `indices` array and the number of vertices
    // in the face.
    std::vector<uint16_t> faces;

    // Face normals.
    std::vector<vector3> normals;

    size_t num_faces() const {
        EDYN_ASSERT(faces.size() % 2 == 0);
        return faces.size() / 2;
    }

    template<typename Func>
    void visit_face(size_t face_idx, Func func) const {
        const auto first = faces[face_idx * 2];
        const auto count = faces[face_idx * 2 + 1];

        for (size_t i = first; i < first + count; ++i) {
            func(vertices[indices[i]]);
        }
    }
    
    void calculate_normals();

    void calculate_edges();
};

}

#endif // EDYN_SHAPES_CONVEX_MESH_HPP
