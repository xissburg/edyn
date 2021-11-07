#include "edyn/util/triangle_util.hpp"
#include "edyn/math/constants.hpp"
#include "edyn/shapes/triangle_mesh.hpp"

namespace edyn {

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

    return (d0 > -EDYN_EPSILON && d1 > -EDYN_EPSILON && d2 > -EDYN_EPSILON) ||
           (d0 < EDYN_EPSILON && d1 < EDYN_EPSILON && d2 < EDYN_EPSILON);
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

scalar get_triangle_support_projection(const triangle_vertices &vertices, const vector3 &dir) {
    auto max_proj = -EDYN_SCALAR_MAX;

    for (size_t i = 0; i < vertices.size(); ++i) {
        const auto &point_local = vertices[i];
        auto proj = dot(point_local, dir);

        if (proj > max_proj) {
            max_proj = proj;
        }
    }

    return max_proj;
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

vector3 clip_triangle_separating_axis(vector3 sep_axis, const triangle_mesh &mesh,
                                      size_t tri_idx, const triangle_vertices &tri_vertices,
                                      const vector3 &tri_normal,triangle_feature tri_feature,
                                      size_t tri_feature_index) {
    // Project separating axis into voronoi region of triangle feature.
    // Return zero if the axis should be ignored, which happens in case the
    // feature is a vertex and the axis does not lie in the voronoi region.
    switch (tri_feature) {
    case triangle_feature::vertex: {
        // Get the normals of faces adjacent to the edges that share this vertex.
        auto edge_idx0 = tri_feature_index;
        auto edge_idx1 = (tri_feature_index + 2) % 3;

        auto adj_normal0 = mesh.get_adjacent_face_normal(tri_idx, edge_idx0);
        auto adj_normal1 = mesh.get_adjacent_face_normal(tri_idx, edge_idx1);

        auto edge_dir0 = tri_vertices[(edge_idx0 + 1) % 3] - tri_vertices[edge_idx0];
        auto edge_normal0 = cross(edge_dir0, adj_normal0);

        auto edge_dir1 = tri_vertices[(edge_idx1 + 1) % 3] - tri_vertices[edge_idx1];
        auto edge_normal1 = cross(edge_dir1, adj_normal1);

        // Test against the edge that is closer to the separating axis. Use the
        // same logic as its done for edges: select the triangle normal if it is
        // concave, or else select the adjacent triangle normal if the separating
        // axis is beyond the voronoi region.
        if (dot(sep_axis, edge_normal0) > dot(sep_axis, edge_normal1)) {
            auto is_convex0 = dot(tri_normal, edge_normal0) < 0;

            if (!is_convex0) {
                sep_axis = tri_normal;
            } else if (dot(sep_axis, edge_normal0) > 0) {
                sep_axis = adj_normal0;
            }
        } else {
            auto is_convex1 = dot(tri_normal, edge_normal1) < 0;

            if (!is_convex1) {
                sep_axis = tri_normal;
            } else if (dot(sep_axis, edge_normal1) > 0) {
                sep_axis = adj_normal1;
            }
        }

        break;
    } case triangle_feature::edge: {
        auto adj_normal = mesh.get_adjacent_face_normal(tri_idx, tri_feature_index);
        auto v0 = tri_vertices[tri_feature_index];
        auto v1 = tri_vertices[(tri_feature_index + 1) % 3];
        auto edge_dir = v1 - v0;
        auto edge_normal = cross(edge_dir, adj_normal); // Points outside.
        auto is_convex = dot(tri_normal, edge_normal) < 0;

        // If this is a concave edge, use the triangle normal. If the axis is
        // outside the voronoi region, use the adjacent normal, which equates
        // to rotating the axis towards the region.
        if (!is_convex) {
            sep_axis = tri_normal;
        } else if (dot(sep_axis, edge_normal) > 0) {
            sep_axis = adj_normal;
        }

        break;
    } case triangle_feature::face: {
        sep_axis = tri_normal;
    }}

    return sep_axis;
}

}
