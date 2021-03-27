#include "edyn/shapes/convex_mesh.hpp"

namespace edyn {

void convex_mesh::calculate_normals() {
    normals.clear();

    for (size_t i = 0; i < num_triangles(); ++i) {
        auto vertices = get_triangle(i);
        auto normal = cross(vertices[1] - vertices[0], vertices[2] - vertices[1]);
        normal = normalize(normal);
        normals.push_back(normal);
    }
}

void convex_mesh::calculate_edges() {
    edges.clear();

    for (size_t i = 0; i < num_triangles(); ++i) {
        for (size_t j = 0; j < 3; ++j) {
            auto contains = false;
            auto i0 = indices[i * 3 + j];
            auto i1 = indices[i * 3 + (j + 1) % 3];

            for (size_t k = 0; k < edges.size(); k += 2) {
                if ((edges[k] == i0 && edges[k + 1] == i1) ||
                    (edges[k] == i1 && edges[k + 1] == i0)) {
                    contains = true;
                    break;
                }
            }

            if (!contains) {
                edges.push_back(i0);
                edges.push_back(i1);
            }
        }
    }
}

}