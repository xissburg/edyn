#ifndef EDYN_UTIL_SHAPE_UTIL_HPP
#define EDYN_UTIL_SHAPE_UTIL_HPP

#include "edyn/math/scalar.hpp"
#include "edyn/math/vector2.hpp"
#include "edyn/math/vector3.hpp"
#include "edyn/math/quaternion.hpp"
#include "edyn/comp/aabb.hpp"
#include <vector>
#include <cstdint>
#include <string>

namespace edyn {

void make_plane_mesh(scalar extent_x, scalar extent_z, 
                     size_t num_vertices_x, size_t num_vertices_z, 
                     std::vector<vector3> &vertices, std::vector<uint16_t> &indices);

bool load_mesh_from_obj(const std::string &path, 
                        std::vector<vector3> &vertices, 
                        std::vector<uint16_t> &indices);

vector3 support_point_box(const vector3 &half_extents, const vector3 &dir);

AABB aabb_of_aabb(const AABB &aabb, const vector3 &pos, const quaternion &orn);

template<typename It>
AABB point_cloud_aabb(It first, It last) {
    // TODO: implement and use `parallel_reduce`.
    auto aabb = AABB{vector3_max, -vector3_max};

    for (auto it = first; it != last; ++it) {
        aabb.min = min(aabb.min, *it);
        aabb.max = max(aabb.max, *it);
    }

    return aabb;
}

template<typename It>
AABB point_cloud_aabb(It first, It last,
                      const vector3 &pos, const quaternion &orn) {
    // TODO: implement and use `parallel_reduce`.
    auto aabb = AABB{vector3_max, -vector3_max};

    for (auto it = first; it != last; ++it) {
        auto point_world = to_world_space(*it, pos, orn);
        aabb.min = min(aabb.min, point_world);
        aabb.max = max(aabb.max, point_world);
    }

    return aabb;
}

template<typename It>
vector3 point_cloud_support_point(It first, It last,
                                  const vector3 &pos, const quaternion &orn,
                                  const vector3 &dir, scalar *projection = nullptr) {
    auto result = vector3_zero;
    auto max_proj = -EDYN_SCALAR_MAX;

    for (auto it = first; it != last; ++it) {
        auto point_world = to_world_space(*it, pos, orn);
        auto proj = dot(point_world, dir);

        if (proj > max_proj) {
            max_proj = proj;
            result = point_world;
        }
    }

    if (projection) {
        *projection = max_proj;
    }

    return result;
}

std::vector<size_t> calculate_convex_hull(const std::vector<vector2> &points, scalar tolerance);

bool point_inside_convex_polygon(const std::vector<vector2> &vertices, const vector2 &point);

}

#endif // EDYN_UTIL_SHAPE_UTIL_HPP