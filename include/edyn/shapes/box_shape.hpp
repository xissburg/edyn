#ifndef EDYN_SHAPES_BOX_SHAPE_HPP
#define EDYN_SHAPES_BOX_SHAPE_HPP

#include "edyn/math/vector3.hpp"
#include "edyn/math/matrix3x3.hpp"
#include "edyn/math/geom.hpp"
#include "edyn/comp/aabb.hpp"
#include <tuple>
#include <array>

namespace edyn {

enum box_feature {
    BOX_FEATURE_VERTEX,
    BOX_FEATURE_EDGE,
    BOX_FEATURE_FACE
};

struct box_shape {
    vector3 half_extents;

    AABB aabb(const vector3 &pos, const quaternion &orn) const;

    vector3 inertia(scalar mass) const;

    vector3 support_point(const vector3 &dir) const;

    vector3 support_point(const quaternion &orn, const vector3 &dir) const;
    
    vector3 support_point(const vector3 &pos, const quaternion &orn, const vector3 &dir) const;

    void support_feature(const vector3 &dir, box_feature &feature, 
                         size_t &feature_index, scalar &projection,
                         scalar threshold) const;

    void support_feature(const vector3 &pos, const quaternion &orn, 
                         const vector3 &axis_pos, const vector3 &axis_dir,
                         box_feature &feature, size_t &feature_index,
                         scalar &projection,
                         scalar threshold) const;

    vector3 get_vertex(size_t i) const;

    vector3 get_vertex(size_t i, const vector3 &pos, const quaternion &orn) const;

    std::array<vector3, 2> get_edge(size_t i) const;

    std::array<vector3, 2> get_edge(size_t i, const vector3 &pos, const quaternion &orn) const;

    std::array<vector3, 4> get_face(size_t i) const;

    std::array<vector3, 4> get_face(size_t i, const vector3 &pos, const quaternion &orn) const;

    vector3 get_face_normal(size_t i) const;

    vector3 get_face_normal(size_t i, const quaternion &orn) const;

    size_t get_edge_index(size_t v0_idx, size_t v1_idx) const;
    size_t get_face_index(size_t v0_idx, size_t v1_idx,
                          size_t v2_idx, size_t v3_idx) const;
};

}

#endif // EDYN_SHAPES_BOX_SHAPE_HPP