#ifndef EDYN_UTIL_SHAPE_UTIL_HPP
#define EDYN_UTIL_SHAPE_UTIL_HPP

#include "edyn/math/scalar.hpp"
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

template<typename VectorIterator>
AABB point_cloud_aabb(VectorIterator vector_begin, VectorIterator vector_end) {
    // TODO: implement and use `parallel_reduce`.
    auto aabb = AABB{vector3_max, -vector3_max};

    for (auto it = vector_begin; it != vector_end; ++it) {
        aabb.min = min(aabb.min, *it);
        aabb.max = max(aabb.max, *it);
    }

    return aabb;
}

template<typename VectorIterator>
AABB point_cloud_aabb(VectorIterator vector_begin, VectorIterator vector_end,
                      const vector3 &pos, const quaternion &orn) {
    // TODO: implement and use `parallel_reduce`.
    auto aabb = AABB{vector3_max, -vector3_max};

    for (auto it = vector_begin; it != vector_end; ++it) {
        auto point_world = to_world_space(*it, pos, orn);
        aabb.min = min(aabb.min, point_world);
        aabb.max = max(aabb.max, point_world);
    }

    return aabb;
}

template<typename VectorIterator>
vector3 point_cloud_support_point(VectorIterator vector_begin, VectorIterator vector_end,
                                  const vector3 &pos, const quaternion &orn,
                                  const vector3 &dir, scalar *projection = nullptr) {
    auto result = vector3_zero;
    auto max_proj = -EDYN_SCALAR_MAX;

    for (auto it = vector_begin; it != vector_end; ++it) {
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

}

#endif // EDYN_UTIL_SHAPE_UTIL_HPP