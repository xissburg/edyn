#include "edyn/shapes/triangle_shape.hpp"
#include "edyn/math/constants.hpp"

namespace edyn {

void triangle_shape::update_computed_properties() {
    edges = get_triangle_edges(vertices);
    normal = normalize(cross(edges[0], edges[1]));

    for (int i = 0; i < 3; ++i) {
        // If edge starting or ending in this vertex are concave, thus is the vertex.
        is_concave_vertex[i] = is_concave_edge[i] || is_concave_edge[(i + 2) % 3];
        edge_tangents[i] = cross(edges[i], normal);
    }
}

bool triangle_shape::ignore_edge(size_t idx, const vector3 &dir) const {
    return is_concave_edge[idx] ||
            dot(dir, edge_tangents[idx]) < -EDYN_EPSILON ||
            dot(dir, normal) < cos_angles[idx];
}

bool triangle_shape::ignore_vertex(size_t idx, const vector3 &dir) const {
    if (is_concave_vertex[idx]) {
        return true;
    }

    // Ignore vertex if the direction is not in the Voronoi region the
    // edges that share it.
    auto dot_tangent_0 = dot(dir, edge_tangents[idx]);
    auto dot_tangent_1 = dot(dir, edge_tangents[(idx + 2) % 3]);

    if (dot_tangent_0 < -EDYN_EPSILON && dot_tangent_1 < -EDYN_EPSILON) {
        return true;
    }

    return dot(dir, normal) < cos_angles[idx];
}

bool triangle_shape::ignore_feature(triangle_feature tri_feature, 
                                    size_t idx, const vector3 &dir) const {
    switch (tri_feature) {
    case triangle_feature::edge:
        return ignore_edge(idx, dir);
    case triangle_feature::vertex:
        return ignore_vertex(idx, dir);
    case triangle_feature::face:
        return false;
    }
    return false;
}

bool point_in_triangle(const triangle_vertices &vertices, 
                       const vector3 &normal, 
                       const vector3 &p) {
    auto edges = get_triangle_edges(vertices);

    auto q0 = p - vertices[0];
    auto q1 = p - vertices[1];
    auto q2 = p - vertices[2];

    auto en0 = cross(edges[0], normal);
    auto en1 = cross(edges[1], normal);
    auto en2 = cross(edges[2], normal);

    auto d0 = dot(en0, q0);
    auto d1 = dot(en1, q1);
    auto d2 = dot(en2, q2);

    return (d0 >= 0 && d1 >= 0 && d2 >= 0) || 
           (d0 <= 0 && d1 <= 0 && d2 <= 0);
}

triangle_edges get_triangle_edges(const triangle_vertices &vertices) {
    return {
        vertices[1] - vertices[0],
        vertices[2] - vertices[1],
        vertices[0] - vertices[2]
    };
}

void get_triangle_support_feature(const triangle_vertices &vertices, 
                                  const vector3 &axis_pos, const vector3 &axis_dir,
                                  triangle_feature &tri_feature,
                                  size_t &tri_feature_index,
                                  scalar &projection, scalar threshold) {
    projection = -large_scalar;

    for (size_t i = 0; i < 3; ++i) {
        auto &v = vertices[i];
        auto proj_i = dot(v - axis_pos, axis_dir);

        // If the projection is near the current maximum, it means 
        // there's another vertex already at that spot, thus the 
        // feature could be either an edge or the face.
        if (i > 0 && std::abs(proj_i - projection) < threshold) {
            // If the maximum feature is a vertex, then the current vertex
            // is included to form an edge.
            if (tri_feature == triangle_feature::vertex) {
                tri_feature = triangle_feature::edge;

                if (i == 2) {
                    // If this is the third vertex (index 2), the previous in this 
                    // for loop could have been either vertex 1 or 0. If 0, then the 
                    // edge is the last one, i.e. edge 2. If 1, then the edge is #1.
                    if (tri_feature_index == 0) {
                        tri_feature_index = 2;
                    } else {
                        tri_feature_index = 1;
                    }
                } else {
                    // If this is the second vertex (index 1), the previous could
                    // only have been vertex 0, thus this must be edge 0.
                    tri_feature_index = 0;
                    projection = std::max(proj_i, projection);
                }
            } else if (tri_feature == triangle_feature::edge) {
                // If the maximum feature was already an edge, adding this
                // vertex to it makes it a face.
                tri_feature = triangle_feature::face;
                projection = std::max(proj_i, projection);
            }
        } else if (proj_i > projection) {
            projection = proj_i;
            tri_feature = triangle_feature::vertex;
            tri_feature_index = i;
        }
    }
}

vector3 get_triangle_support_point(const triangle_vertices &vertices, const vector3 &dir) {
    auto sup = vector3_zero;
    auto max_proj = -EDYN_SCALAR_MAX;

    for (size_t i = 0; i < vertices.size(); ++i) {
        const auto &point_local = vertices[i];
        auto proj = dot(point_local, dir);

        if (proj > max_proj) {
            max_proj = proj;
            sup = point_local;
        }
    }

    return sup;
}

size_t get_triangle_feature_num_vertices(triangle_feature feature) {
    return static_cast<size_t>(feature) + 1;
}

size_t get_triangle_feature_num_edges(triangle_feature feature) {
    if (feature == triangle_feature::edge) {
        return 1;
    } else if (feature == triangle_feature::face) {
        return 3;
    }
    return 0;
}

AABB get_triangle_aabb(const triangle_vertices &vertices) {
    auto tri_min = min(min(vertices[0], vertices[1]), vertices[2]);
    auto tri_max = max(max(vertices[0], vertices[1]), vertices[2]);
    return {tri_min, tri_max};
}

}