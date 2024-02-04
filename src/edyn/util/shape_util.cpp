#include "edyn/util/shape_util.hpp"
#include "edyn/config/config.h"
#include "edyn/math/quaternion.hpp"
#include "edyn/math/scalar.hpp"
#include "edyn/math/vector2.hpp"
#include "edyn/math/math.hpp"
#include "edyn/math/vector3.hpp"
#include "edyn/shapes/triangle_mesh.hpp"

namespace edyn {

void make_box_mesh(const vector3 &he,
                   std::vector<vector3> &vertices,
                   std::vector<uint32_t> &indices,
                   std::vector<uint32_t> &faces) {
    vertices.push_back({-he.x, -he.y, -he.z});
    vertices.push_back({ he.x, -he.y, -he.z});
    vertices.push_back({ he.x, -he.y,  he.z});
    vertices.push_back({-he.x, -he.y,  he.z});
    vertices.push_back({-he.x,  he.y, -he.z});
    vertices.push_back({ he.x,  he.y, -he.z});
    vertices.push_back({ he.x,  he.y,  he.z});
    vertices.push_back({-he.x,  he.y,  he.z});

    indices.insert(indices.end(), {0, 1, 2, 3}); // bottom
    indices.insert(indices.end(), {7, 6, 5, 4}); // top
    indices.insert(indices.end(), {4, 5, 1, 0}); // front
    indices.insert(indices.end(), {6, 7, 3, 2}); // rear
    indices.insert(indices.end(), {7, 4, 0, 3}); // left
    indices.insert(indices.end(), {5, 6, 2, 1}); // right

    faces.insert(faces.end(), {0, 4});
    faces.insert(faces.end(), {4, 4});
    faces.insert(faces.end(), {8, 4});
    faces.insert(faces.end(), {12, 4});
    faces.insert(faces.end(), {16, 4});
    faces.insert(faces.end(), {20, 4});
}

vector3 support_point_box(const vector3 &half_extents, const vector3 &dir) {
    return {
        dir.x > 0 ? half_extents.x : -half_extents.x,
        dir.y > 0 ? half_extents.y : -half_extents.y,
        dir.z > 0 ? half_extents.z : -half_extents.z
    };
}

scalar polyhedron_support_projection(const std::vector<vector3> &vertices,
                                     const std::vector<uint32_t> &neighbors_start,
                                     const std::vector<uint32_t> &neighbor_indices,
                                     const vector3 &dir) {
    // Starting at the first vertex, visit all neighbors and pick the neighboring
    // vertex with higher projection. Stop when there are no more neighbors with
    // a higher projection value than the current.
    auto v_idx = 0u;
    auto max_proj = dot(vertices[0], dir);

    EDYN_ASSERT(neighbors_start.size() == vertices.size() + 1);

    while (true) {
        auto n_idx0 = neighbors_start[v_idx];
        auto n_idx1 = neighbors_start[v_idx + 1];
        bool done = true;

        for (auto i = n_idx0; i < n_idx1; ++i) {
            auto nv_idx = neighbor_indices[i];
            auto proj = dot(vertices[nv_idx], dir);

            if (proj > max_proj) {
                max_proj = proj;
                v_idx = nv_idx;
                done = false;
            }
        }

        if (done) {
            break;
        }
    }

    return max_proj;
}

vector3 point_cloud_support_point(const std::vector<vector3> &points, const vector3 &dir) {
    return point_cloud_support_point(points.begin(), points.end(), dir);
}

scalar point_cloud_support_projection(const std::vector<vector3> &points, const vector3 &dir) {
    return point_cloud_support_projection(points.begin(), points.end(), dir);
}

size_t split_hull_edge(const std::vector<vector2> &points,
                       std::vector<size_t> &hull,
                       size_t i0, size_t i1, scalar tolerance) {

    auto v0 = points[hull[i0]];
    auto v1 = points[hull[i1]];
    auto edge = v1 - v0;
    // Hull vertices are oriented counter-clockwise. Rotate edge clockwise
    // to obtain a vector that points outside the convex polygon.
    auto dir = -orthogonal(edge);
    auto max_proj = -EDYN_SCALAR_MAX;
    auto idx = size_t{};

    for (size_t i = 0; i < points.size(); ++i) {
        auto &p = points[i];
        auto proj = dot(p, dir);

        if (proj > max_proj) {
            max_proj = proj;
            idx = i;
        }
    }

    if (dot(points[idx] - v0, dir) > tolerance) {
        hull.insert(hull.begin() + i1, idx);
        auto num_splits = split_hull_edge(points, hull, i0, i1, tolerance);

        i1 += num_splits;
        auto i2 = i1 + 1;
        num_splits += split_hull_edge(points, hull, i1, i2, tolerance);

        return 1 + num_splits;
    }

    return 0;
}

std::vector<size_t> calculate_convex_hull(const std::vector<vector2> &points, scalar tolerance) {
    if (points.size() <= 3) {
        if (points.size() == 3) {
            // It is a triangle, just have to make sure vertices are
            // oriented counter-clockwise.
            if (is_triangle_ccw(points[0], points[1], points[2])) {
                return {0, 1, 2};
            } else {
                return {2, 1, 0};
            }
        } else if (points.size() == 2) {
            return {0, 1};
        } else {
            return {0};
        }
    }

    // Quickhull algorithm.
    auto pt_min = vector2_one * EDYN_SCALAR_MAX;
    auto pt_max = vector2_one * -EDYN_SCALAR_MAX;
    auto pt_min_idx = size_t{};
    auto pt_max_idx = size_t{};

    for (size_t i = 0; i < points.size(); ++i) {
        auto &p = points[i];

        if (p.x < pt_min.x) {
            pt_min = p;
            pt_min_idx = i;
        }
        if (p.x > pt_max.x) {
            pt_max = p;
            pt_max_idx = i;
        }
    }

    auto hull = std::vector<size_t>();

    if (pt_max.x - pt_min.x < tolerance) {
        // Point set is a vertical sliver. Return a vertical segment.
        pt_min = vector2_one * EDYN_SCALAR_MAX;
        pt_max = vector2_one * -EDYN_SCALAR_MAX;

        for (size_t i = 0; i < points.size(); ++i) {
            auto &p = points[i];

            if (p.y < pt_min.y) {
                pt_min = p;
                pt_min_idx = i;
            }
            if (p.y > pt_max.y) {
                pt_max = p;
                pt_max_idx = i;
            }
        }

        hull.push_back(pt_max_idx);
        hull.push_back(pt_min_idx);
        return hull;
    }

    hull.push_back(pt_max_idx);
    hull.push_back(pt_min_idx);
    hull.push_back(pt_max_idx);

    size_t i0 = 0;
    size_t i1 = 1;
    auto num_splits = split_hull_edge(points, hull, i0, i1, tolerance);

    i1 += num_splits;
    auto i2 = i1 + 1;
    split_hull_edge(points, hull, i1, i2, tolerance);

    hull.pop_back();

    return hull;
}

bool point_inside_convex_polygon(const std::vector<vector2> &vertices, const vector2 &point) {
    EDYN_ASSERT(vertices.size() > 2);

    for (size_t i = 0; i < vertices.size() - 1; ++i) {
        auto d = point - vertices[i];
        auto e = vertices[i + 1] - vertices[i];
        // Vertices are oriented counter-clockwise. Rotate edge clockwise
        // to obtain a vector that points outside the convex polygon.
        auto n = -orthogonal(e);

        if (dot(d, n) > 0) {
            return false;
        }
    }

    return true;
}

bool is_triangle_ccw(const vector2 &v0, const vector2 &v1, const vector2 &v2) {
    auto e = v1 - v0;
    auto t = orthogonal(e);
    return dot(v2 - v0, t) > 0;
}

bool sort_triangle_ccw(vector2 &v0, vector2 &v1, vector2 &v2) {
    if (!is_triangle_ccw(v0, v1, v2)) {
        std::swap(v0, v2);
        return true;
    }

    return false;
}

bool closest_point_convex_polygon(const std::vector<vector2> &vertices,
                                  const std::vector<size_t> &indices,
                                  const vector2 &p, vector2 &closest) {
    // Find Voronoi region which contains `p`.
    for (size_t i = 0; i < indices.size(); ++i) {
        auto j = (i + 1) % indices.size();
        auto i0 = indices[i];
        auto i1 = indices[j];
        auto &v0 = vertices[i0];
        auto &v1 = vertices[i1];

        auto e0 = v1 - v0;
        // Vertices are oriented counter-clockwise. Rotate edge clockwise
        // to obtain a vector that points outside the convex polygon.
        auto n0 = -orthogonal(e0);

        if (dot(p - v0, n0) < 0) {
            // Point is behind this edge.
            continue;
        }

        if (dot(p - v0, e0) > 0) {
            // Point is past the first boundary of the Voronoi region of
            // this edge.
            if (dot(p - v1, e0) < 0) {
                // Point is before the second boundary of the Voronoi region
                // of this edge, thus this is the Voronoi region where it belongs.
                auto t = dot(p - v0, e0) / dot(e0, e0);
                closest = lerp(v0, v1, t);
                return true;
            } else {
                // Point is past the second boundary of the Voronoi region
                // of this edge. If it's also before the first boundary of
                // the Voronoi region on the next edge, it means the Voronoi
                // region of the vertex connecting these two edges contains
                // the point.
                auto k = (i + 2) % indices.size();
                auto i2 = indices[k];
                auto &v2 = vertices[i2];
                auto e1 = v2 - v1;

                if (dot(p - v1, e1) < 0) {
                    closest = v1;
                    return true;
                }
            }
        }
    }

    return false;
}

bool closest_point_polygon(const support_polygon &polygon,
                           const vector2 &p, vector2 &closest) {
    return closest_point_convex_polygon(polygon.plane_vertices, polygon.hull, p, closest);
}

scalar capsule_support_projection(const vector3 &v0, const vector3 &v1,
                                  scalar radius, const vector3 &dir) {
    return std::max(dot(v0, dir), dot(v1, dir)) + radius;
}

scalar capsule_support_projection(const std::array<vector3, 2> &vertices,
                                  scalar radius, const vector3 &dir) {
    return capsule_support_projection(vertices[0], vertices[1], radius, dir);
}

vector3 cylinder_support_point(scalar radius, scalar half_length, coordinate_axis axis, const vector3 &dir) {
    // Index of vector element in cylinder object space that represents the
    // cylinder axis.
    auto cyl_ax_idx = static_cast<std::underlying_type_t<coordinate_axis>>(axis);
    // Index of vector elements orthogonal to cylinder axis.
    auto cyl_ax_ortho_idx0 = (cyl_ax_idx + 1) % 3;
    auto cyl_ax_ortho_idx1 = (cyl_ax_idx + 2) % 3;

    // Squared length in plane orthogonal to cylinder axis.
    auto planar_len_sq = dir[cyl_ax_ortho_idx0] * dir[cyl_ax_ortho_idx0] + dir[cyl_ax_ortho_idx1] * dir[cyl_ax_ortho_idx1];
    vector3 sup;
    sup[cyl_ax_idx] = dir[cyl_ax_idx] < 0 ? -half_length : half_length;

    if (planar_len_sq > EDYN_EPSILON) {
        auto d = radius / std::sqrt(planar_len_sq);
        sup[cyl_ax_ortho_idx0] = dir[cyl_ax_ortho_idx0] * d;
        sup[cyl_ax_ortho_idx1] = dir[cyl_ax_ortho_idx1] * d;
    } else {
        sup[cyl_ax_ortho_idx0] = radius;
        sup[cyl_ax_ortho_idx1] = 0;
    }

    return sup;
}

vector3 cylinder_support_point(scalar radius, scalar half_length, coordinate_axis axis,
                               const quaternion &orn, const vector3 &dir) {
    auto local_dir = rotate(conjugate(orn), dir);
    auto pt = cylinder_support_point(radius, half_length, axis, local_dir);
    return rotate(orn, pt);
}

vector3 cylinder_support_point(scalar radius, scalar half_length, coordinate_axis axis,
                               const vector3 &pos, const quaternion &orn, const vector3 &dir) {
    return pos + cylinder_support_point(radius, half_length, axis, orn, dir);
}

scalar cylinder_support_projection(scalar radius, scalar half_length, coordinate_axis axis,
                                   const vector3 &pos, const quaternion &orn, const vector3 &dir) {
    auto local_dir = rotate(conjugate(orn), dir);
    auto pt = cylinder_support_point(radius, half_length, axis, local_dir);
    return dot(pos, dir) + dot(pt, local_dir);
}

vector3 mesh_centroid(const std::vector<vector3> &vertices,
                      const std::vector<uint32_t> &indices,
                      const std::vector<uint32_t> &faces) {
    // Reference: "Calculating the volume and centroid of a polyhedron in 3d"
    // http://wwwf.imperial.ac.uk/~rn/centroid.pdf
    auto center = vector3_zero;
    auto volume = scalar(0);

    EDYN_ASSERT(faces.size() % 2 == 0);

    for (size_t i = 0; i < faces.size(); i += 2) {
        auto first = faces[i];
        auto count = faces[i + 1];
        EDYN_ASSERT(count >= 3);

        auto i0 = indices[first];
        auto &v0 = vertices[i0];

        // Triangulate face with a triangle fan around v0.
        for (size_t j = 1; j < size_t(count - 1); ++j) {
            auto i1 = indices[first + j];
            auto i2 = indices[first + j + 1];
            auto &v1 = vertices[i1];
            auto &v2 = vertices[i2];
            auto normal = cross(v1 - v0, v2 - v1);

            // Six times the signed tetrahedron volume.
            auto tet_vol = dot(v0, normal);
            volume += tet_vol;

            auto vx = vector3{v0.x + v1.x, v1.x + v2.x, v2.x + v0.x};
            auto vy = vector3{v0.y + v1.y, v1.y + v2.y, v2.y + v0.y};
            auto vz = vector3{v0.z + v1.z, v1.z + v2.z, v2.z + v0.z};
            auto w = vector3{length_sqr(vx), length_sqr(vy), length_sqr(vz)};
            center += normal * w;
        }
    }

    volume /= 6;
    center /= 24 * 2 * volume;

    return center;
}

size_t get_triangle_mesh_feature_index(const triangle_mesh &mesh, size_t tri_idx,
                                       triangle_feature tri_feature, size_t tri_feature_idx) {
    switch (tri_feature) {
    case triangle_feature::face:
        return tri_idx;
    case triangle_feature::edge:
        return mesh.get_face_edge_index(tri_idx, tri_feature_idx);
    case triangle_feature::vertex:
        return mesh.get_face_vertex_index(tri_idx, tri_feature_idx);
    }

    return SIZE_MAX;
}

}
