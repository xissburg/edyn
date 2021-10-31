#include "edyn/util/shape_util.hpp"
#include "edyn/math/quaternion.hpp"
#include "edyn/math/scalar.hpp"
#include "edyn/math/vector2.hpp"
#include "edyn/math/math.hpp"
#include "edyn/math/vector3.hpp"
#include "edyn/shapes/triangle_mesh.hpp"
#include <fstream>
#include <sstream>
#include <numeric>

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

static vector3 read_vector3(std::istringstream &iss) {
    edyn::vector3 v;
    iss >> v.x >> v.y >> v.z;
    return v;
}

static void read_face_indices(std::istringstream &iss,
                              std::vector<uint32_t> &indices,
                              uint32_t offset, bool triangulate) {
    std::string idx_str;
    auto count = size_t{};
    uint32_t first_idx;
    uint32_t prev_idx;

    while (true) {
        iss >> idx_str;
        if (iss.fail()) break;

        // Remove everything after the first slash to keep only the first
        // element in the "v/vt/vn" sequence.
        auto pos = idx_str.find_first_of('/');
        if (pos != std::string::npos) {
            idx_str.erase(pos);
        }

        auto idx = static_cast<uint32_t>(std::stoi(idx_str));
        EDYN_ASSERT(idx >= 1 + offset);

        if (triangulate && count >= 3) {
            indices.push_back(first_idx);
            indices.push_back(prev_idx);
        }

        indices.push_back(idx - 1 - offset);

        if (count == 0) {
            first_idx = indices.back();
        }

        prev_idx = indices.back();
        ++count;
    }
}

static void read_face(std::istringstream &iss,
                      std::vector<uint32_t> &indices,
                      std::vector<uint32_t> &faces,
                      uint32_t offset, bool triangulate) {
    // Store where this face starts in the `indices` array.
    faces.push_back(indices.size());

    read_face_indices(iss, indices, offset, triangulate);

    // Store the number of vertices in this face.
    auto count = indices.size() - faces.back();
    faces.push_back(count);
}

bool load_meshes_from_obj(const std::string &path,
                          std::vector<obj_mesh> &meshes,
                          vector3 pos,
                          quaternion orn,
                          vector3 scale) {
    auto file = std::ifstream(path);

    if (!file.is_open()) {
        return false;
    }

    std::string line;
    auto mesh = obj_mesh{};
    uint32_t index_offset = 0;

    while (std::getline(file, line)) {
        auto pos_space = line.find(" ");

        if (pos_space == std::string::npos) {
            continue;
        }

        auto cmd = line.substr(0, pos_space);

        if (cmd == "o") {
            if (!mesh.vertices.empty()) {
                index_offset += mesh.vertices.size();
                meshes.emplace_back(std::move(mesh));
            }
        } else if (cmd == "v") {
            auto iss = std::istringstream(line.substr(pos_space, line.size() - pos_space));
            auto v = read_vector3(iss);

            if (scale != vector3_one) {
                v *= scale;
            }

            if (orn != quaternion_identity) {
                v = rotate(orn, v);
            }

            if (pos != vector3_zero ) {
                v += pos;
            }

            mesh.vertices.push_back(v);

            if (!iss.eof()) {
                // Try reading vertex color.
                auto color = read_vector3(iss);

                if (!iss.fail()) {
                    mesh.colors.push_back(color);
                }
            }
        } else if (cmd == "f") {
            auto iss = std::istringstream(line.substr(pos_space, line.size() - pos_space));
            read_face(iss, mesh.indices, mesh.faces, index_offset, false);
        }
    }

    if (!mesh.vertices.empty()) {
        meshes.emplace_back(std::move(mesh));
    }

    return true;
}

bool load_tri_mesh_from_obj(const std::string &path,
                        std::vector<vector3> &vertices,
                        std::vector<uint32_t> &indices,
                        std::vector<vector3> *colors,
                        vector3 pos,
                        quaternion orn,
                        vector3 scale) {
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
            auto v = read_vector3(iss);

            if (scale != vector3_one) {
                v *= scale;
            }

            if (orn != quaternion_identity) {
                v = rotate(orn, v);
            }

            if (pos != vector3_zero ) {
                v += pos;
            }

            vertices.push_back(v);

            if (colors != nullptr && !iss.eof()) {
                // Try reading vertex color.
                auto color = read_vector3(iss);

                if (!iss.fail()) {
                    colors->push_back(color);
                }
            }
        } else if (cmd == "f") {
            auto iss = std::istringstream(line.substr(pos_space, line.size() - pos_space));
            read_face_indices(iss, indices, 0, true);
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

vector3 cylinder_support_point(scalar radius, scalar half_length, const vector3 &dir) {
    // Squared length in yz plane.
    auto lyz2 = dir.y * dir.y + dir.z * dir.z;

    if (lyz2 > EDYN_EPSILON) {
        auto d = radius / std::sqrt(lyz2);
        return {dir.x < 0 ? -half_length : half_length, dir.y * d, dir.z * d};
    }

    return {dir.x < 0 ? -half_length : half_length, radius, 0};
}

vector3 cylinder_support_point(scalar radius, scalar half_length,
                               const quaternion &orn, const vector3 &dir) {
    auto local_dir = rotate(conjugate(orn), dir);
    auto pt = cylinder_support_point(radius, half_length, local_dir);
    return rotate(orn, pt);
}

vector3 cylinder_support_point(scalar radius, scalar half_length, const vector3 &pos,
                               const quaternion &orn, const vector3 &dir) {
    return pos + cylinder_support_point(radius, half_length, orn, dir);
}

scalar cylinder_support_projection(scalar radius, scalar half_length, const vector3 &pos,
                                   const quaternion &orn, const vector3 &dir) {
    auto local_dir = rotate(conjugate(orn), dir);
    auto pt = cylinder_support_point(radius, half_length, local_dir);
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
