#ifndef EDYN_SHAPES_BOX_SHAPE_HPP
#define EDYN_SHAPES_BOX_SHAPE_HPP

#include "edyn/math/vector3.hpp"
#include "edyn/math/matrix3x3.hpp"
#include "edyn/math/geom.hpp"
#include "edyn/comp/aabb.hpp"
#include <tuple>

namespace edyn {

enum box_feature {
    BOX_FEATURE_VERTEX,
    BOX_FEATURE_EDGE,
    BOX_FEATURE_FACE
};

struct box_shape {
    vector3 half_extents;

    AABB aabb(const vector3 &pos, const quaternion &orn) const {
        // Reference: Real-Time Collision Detection - Christer Ericson, section 4.2.6.
        auto aabb = AABB{pos, pos};
        auto basis = to_matrix3x3(orn);

        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                auto e = basis[i][j] * -half_extents[j];
                auto f = -e;

                if (e < f) {
                    aabb.min[i] += e;
                    aabb.max[i] += f;
                } else {
                    aabb.min[i] += f;
                    aabb.max[i] += e;
                }
            }
        }

        return aabb;
    }

    vector3 inertia(scalar mass) const {
        auto extents = half_extents * 2;
        return scalar(1) / scalar(12) * mass * vector3{
            extents.y * extents.y + extents.z * extents.z,
            extents.z * extents.z + extents.x * extents.x,
            extents.x * extents.x + extents.y * extents.y,
        };
    }

    vector3 support_point(const vector3 &dir) const {
        return {
            dir.x > 0 ? half_extents.x : -half_extents.x,
            dir.y > 0 ? half_extents.y : -half_extents.y,
            dir.z > 0 ? half_extents.z : -half_extents.z
        };
    }

    vector3 support_point(const quaternion &orn, const vector3 &dir) const {
        auto local_dir = rotate(conjugate(orn), dir);
        auto pt = support_point(local_dir);
        return rotate(orn, pt);
    }
    
    vector3 support_point(const vector3 &pos, const quaternion &orn, const vector3 &dir) const {
        return pos + support_point(orn, dir);
    }

    std::tuple<box_feature, size_t> support_feature(const vector3 &dir) const;

    std::tuple<box_feature, size_t> support_feature(const quaternion &orn, const vector3 &dir) const {
        auto local_dir = rotate(conjugate(orn), dir);
        return support_feature(local_dir);
    }

    vector3 get_vertex(size_t i) const;

    std::tuple<vector3, vector3> get_edge(size_t i) const;

    std::tuple<vector3, vector3, vector3, vector3>
    get_face(size_t i) const;
};

}

#endif // EDYN_SHAPES_BOX_SHAPE_HPP