#include "edyn/util/shape_util.hpp"
#include "edyn/math/quaternion.hpp"
#include "edyn/math/scalar.hpp"
#include "edyn/math/vector2.hpp"
#include "edyn/math/math.hpp"
#include <fstream>
#include <sstream>
#include <numeric>

namespace edyn {

void make_plane_mesh(scalar extent_x, scalar extent_z, 
                     size_t num_vertices_x, size_t num_vertices_z, 
                     std::vector<vector3> &vertices, std::vector<uint16_t> &indices) {
    const auto half_extent_x = extent_x * scalar(0.5);
    const auto half_extent_z = extent_z * scalar(0.5);
    const auto quad_size_x = extent_x / (num_vertices_x - 1);
    const auto quad_size_z = extent_z / (num_vertices_z - 1);

    for (size_t j = 0; j < num_vertices_z; ++j) {
        auto z = -half_extent_z + j * quad_size_z;
        for (size_t i = 0; i < num_vertices_x; ++i) {
            vertices.push_back({-half_extent_x + i * quad_size_x, 0, z});
        }
    }

    for (size_t j = 0; j < num_vertices_z - 1; ++j) {
        for (size_t i = 0; i < num_vertices_x - 1; ++i) {
            indices.push_back(j * num_vertices_x + i + 1);
            indices.push_back(j * num_vertices_x + i);
            indices.push_back((j + 1) * num_vertices_x + i);

            indices.push_back((j + 1) * num_vertices_x + i);
            indices.push_back((j + 1) * num_vertices_x + i + 1);
            indices.push_back(j * num_vertices_x + i + 1);
        }
    }
}

bool load_mesh_from_obj(const std::string &path, 
                        std::vector<vector3> &vertices, 
                        std::vector<uint16_t> &indices,
                        std::vector<uint16_t> &faces) {
    auto file = std::ifstream(path);

    if (!file.is_open()) {
        return false;
    }

    std::string line;

    while (std::getline(file, line)) {
        auto pos_space = line.find(" ");

        if (pos_space == std::string::npos) {
            continue;
        }

        auto cmd = line.substr(0, pos_space);

        if (cmd == "v") {
            auto iss = std::istringstream(line.substr(pos_space, line.size() - pos_space));
            edyn::vector3 position;
            iss >> position.x >> position.y >> position.z;
            vertices.push_back(position);
        } else if (cmd == "f") {
            // Store where this face starts in the `indices` array.
            faces.push_back(indices.size());

            auto iss = std::istringstream(line.substr(pos_space, line.size() - pos_space));
            uint16_t idx;

            while (true) {
                iss >> idx;
                if (iss.fail()) break;
                indices.push_back(idx - 1);
            }

            // Store the number of vertices in this face.
            auto count = indices.size() - faces.back();
            faces.push_back(count);
        }
    }

    return true;
}

bool load_tri_mesh_from_obj(const std::string &path, 
                        std::vector<vector3> &vertices, 
                        std::vector<uint16_t> &indices) {
    auto file = std::ifstream(path);

    if (!file.is_open()) {
        return false;
    }

    std::string line;

    while (std::getline(file, line)) {
        auto pos_space = line.find(" ");

        if (pos_space == std::string::npos) {
            continue;
        }

        auto cmd = line.substr(0, pos_space);

        if (cmd == "v") {
            auto iss = std::istringstream(line.substr(pos_space, line.size() - pos_space));
            edyn::vector3 position;
            iss >> position.x >> position.y >> position.z;
            vertices.push_back(position);
        } else if (cmd == "f") {
            auto iss = std::istringstream(line.substr(pos_space, line.size() - pos_space));
            uint16_t idx[3];
            iss >> idx[0] >> idx[1] >> idx[2];
            indices.push_back(idx[0] - 1);
            indices.push_back(idx[1] - 1);
            indices.push_back(idx[2] - 1);
        }
    }

    return true;
}

vector3 support_point_box(const vector3 &half_extents, const vector3 &dir) {
    return {
        dir.x > 0 ? half_extents.x : -half_extents.x,
        dir.y > 0 ? half_extents.y : -half_extents.y,
        dir.z > 0 ? half_extents.z : -half_extents.z
    };
}

AABB aabb_of_aabb(const AABB &aabb, const vector3 &pos, const quaternion &orn) {
    auto center = (aabb.min + aabb.max) * scalar(0.5);
    auto center_world = to_world_space(center, pos, orn);
    auto extents = aabb.max - aabb.min;
    auto half_extents = extents * scalar(0.5);

    // Compute support points of the global axes.
    auto c_orn = conjugate(orn);
    auto axis_x = rotate(c_orn, vector3_x);
    auto axis_y = rotate(c_orn, vector3_y);
    auto axis_z = rotate(c_orn, vector3_z);

    auto sup_x = support_point_box(half_extents, axis_x);
    auto sup_y = support_point_box(half_extents, axis_y);
    auto sup_z = support_point_box(half_extents, axis_z);

    auto result_half_extent = vector3{
        dot(sup_x, axis_x),
        dot(sup_y, axis_y),
        dot(sup_z, axis_z)
    };
    auto result_min = center_world - result_half_extent;
    auto result_max = center_world + result_half_extent;
    return {result_min, result_max};
}

AABB point_cloud_aabb(const std::vector<vector3> &points) {
    // TODO: implement and use `parallel_reduce`.
    auto aabb = AABB{vector3_max, -vector3_max};

    for (auto &point : points) {
        aabb.min = min(aabb.min, point);
        aabb.max = max(aabb.max, point);
    }

    return aabb;
}

AABB point_cloud_aabb(const std::vector<vector3> &points, 
                      const vector3 &pos, const quaternion &orn) {
    // TODO: implement and use `parallel_reduce`.
    auto aabb = AABB{vector3_max, -vector3_max};

    for (auto &point_local : points) {
        auto point_world = to_world_space(point_local, pos, orn);
        aabb.min = min(aabb.min, point_world);
        aabb.max = max(aabb.max, point_world);
    }

    return aabb;
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
    auto point = vector2_zero;
    auto idx = size_t{};

    for (size_t i = 0; i < points.size(); ++i) {
        auto &p = points[i];
        auto proj = dot(p, dir);
        if (proj > max_proj) {
            max_proj = proj;
            point = p;
            idx = i;
        }
    }

    if (dot(point - v0, dir) > tolerance) {
        hull.insert(hull.begin() + i1, idx);
        auto num_splits = split_hull_edge(points, hull, i0, i1, tolerance);

        i1 += num_splits;
        auto i2 = i1 + 1;
        num_splits += split_hull_edge(points, hull, i1, i2, tolerance);

        return 1 + num_splits;
    }

    return 0;
}

std::vector<size_t> calculate_convex_hull(std::vector<vector2> &points, scalar tolerance) {
    if (points.size() <= 3) {
        if (points.size() == 3) {
            // It is a triangle, just have to make sure vertices are
            // oriented counter-clockwise.
            sort_triangle_ccw(points[0], points[1], points[2]);
        }

        std::vector<size_t> hull(points.size());
        std::iota(hull.begin(), hull.end(), 0);
        return hull;
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

void sort_triangle_ccw(vector2 &v0, vector2 &v1, vector2 &v2) {
    auto e = v1 - v0;
    auto t = orthogonal(e);

    if (dot(v2 - v0, t) < 0) {
        std::swap(v0, v2);
    }
}

bool closest_point_convex_polygon(const std::vector<vector2> &vertices, 
                                  const std::vector<size_t> &indices, 
                                  const vector2 &p, vector2 &closest) {
    for (auto i = 0; i < indices.size(); ++i) {
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
            continue;
        }

        if (dot(p - v0, e0) > 0) {
            if (dot(p - v1, e0) < 0) {
                auto t = dot(p - v0, e0) / dot(e0, e0);
                closest = lerp(v0, v1, t);
                return true;
            } else {
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

}
