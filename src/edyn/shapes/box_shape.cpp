#include "edyn/shapes/box_shape.hpp"
#include <map>
#include <unordered_set>

namespace edyn {

void box_shape::support_feature(const vector3 &dir, box_feature &feature, 
                                size_t &feature_index, scalar &projection,
                                scalar threshold) const {
    std::array<scalar, 8> projections;
    size_t max_idx;
    projection = -EDYN_SCALAR_MAX;

    for (size_t i = 0; i < 8; ++i) {
        auto v = get_vertex(i);
        auto proj = dot(v, dir);
        projections[i] = proj;

        if (proj > projection) {
            projection = proj;
            max_idx = i;
        }
    }

    std::array<size_t, 4> indices;
    size_t count = 0;

    for (size_t i = 0; i < 8; ++i) {
        if (projections[i] > projection - threshold) {
            indices[count++] = i;

            if (count == 4) {
                break;
            }
        }
    }

    EDYN_ASSERT(count > 0);

    if (count == 1) {
        feature = BOX_FEATURE_VERTEX;
        feature_index = indices[0];
    } else if (count == 2) {
        feature = BOX_FEATURE_EDGE;
        feature_index = get_edge_index(indices[0], indices[1]);
    } else if (count == 3) {
        feature = BOX_FEATURE_EDGE;
        // Select 2 points among the 3 with the higher projections.
        auto proj0 = projections[indices[0]];
        auto proj1 = projections[indices[1]];
        auto proj2 = projections[indices[2]];

        if (proj0 < proj1 && proj0 < proj2) {
            feature_index = get_edge_index(indices[1], indices[2]);
        } else if (proj1 < proj0 && proj1 < proj2) {
            feature_index = get_edge_index(indices[0], indices[2]);
        } else { // if (proj2 > proj0 && proj2 > proj1) {
            feature_index = get_edge_index(indices[0], indices[1]);
        }
    } else {
        feature = BOX_FEATURE_FACE;
        feature_index = get_face_index(indices[0], indices[1], indices[2], indices[3]);
    }
}

void box_shape::support_feature(const vector3 &pos, const quaternion &orn, 
                                const vector3 &axis_pos, const vector3 &axis_dir,
                                box_feature &feature, size_t &feature_index,
                                scalar &projection, scalar threshold) const {
    auto local_dir = rotate(conjugate(orn), axis_dir);
    support_feature(local_dir, feature, feature_index, projection, threshold);
    projection += dot(pos - axis_pos, axis_dir);
}

vector3 box_shape::get_vertex(size_t i) const {
    EDYN_ASSERT(i < 8);
    constexpr vector3 multipliers[] = {
        { 1,  1,  1},
        { 1, -1,  1},
        { 1, -1, -1},
        { 1,  1, -1},
        {-1,  1,  1},
        {-1,  1, -1},
        {-1, -1, -1},
        {-1, -1,  1}
    };
    return half_extents * multipliers[i];
}

vector3 box_shape::get_vertex(size_t i, const vector3 &pos, const quaternion &orn) const {
    return pos + rotate(orn, get_vertex(i));
}

std::array<vector3, 2> box_shape::get_edge(size_t i) const {
    EDYN_ASSERT(i < 12);
    constexpr size_t indices[] = {
        0, 1,
        1, 2,
        2, 3,
        3, 0,
        4, 5,
        5, 6,
        6, 7,
        7, 4,
        0, 4,
        1, 7,
        2, 6,
        3, 5
    };
    return {
        get_vertex(indices[i * 2 + 0]),
        get_vertex(indices[i * 2 + 1])
    };
}

std::array<vector3, 2> box_shape::get_edge(size_t i, const vector3 &pos, const quaternion &orn) const {
    auto edge_vertices = get_edge(i);
    return {
        pos + rotate(orn, edge_vertices[0]),
        pos + rotate(orn, edge_vertices[1])
    };
}

std::array<vector3, 4> box_shape::get_face(size_t i) const {
    EDYN_ASSERT(i < 6);
    constexpr size_t indices[] = {
        0, 1, 2, 3,
        4, 5, 6, 7,
        0, 3, 5, 4,
        1, 7, 6, 2,
        0, 4, 7, 1,
        3, 2, 6, 5
    };
    return {
        get_vertex(indices[i * 4 + 0]),
        get_vertex(indices[i * 4 + 1]),
        get_vertex(indices[i * 4 + 2]),
        get_vertex(indices[i * 4 + 3])
    };
}

std::array<vector3, 4> box_shape::get_face(size_t i, const vector3 &pos, const quaternion &orn) const {
    auto face_vertices = get_face(i);
    return {
        pos + rotate(orn, face_vertices[0]),
        pos + rotate(orn, face_vertices[1]),
        pos + rotate(orn, face_vertices[2]),
        pos + rotate(orn, face_vertices[3])
    };
}

vector3 box_shape::get_face_normal(size_t i) const {
    EDYN_ASSERT(i < 6);
    constexpr vector3 normals[] = {
        { 1,  0,  0},
        {-1,  0,  0},
        { 0,  1,  0},
        { 0, -1,  0},
        { 0,  0,  1},
        { 0,  0, -1}
    };
    return normals[i];
}

vector3 box_shape::get_face_normal(size_t i, const quaternion &orn) const {
    return rotate(orn, get_face_normal(i));
}

size_t box_shape::get_edge_index(size_t v0_idx, size_t v1_idx) const {
    constexpr size_t indices[] = {
        0, 1,
        1, 2,
        2, 3,
        3, 0,
        4, 5,
        5, 6,
        6, 7,
        7, 4,
        0, 4,
        1, 7,
        2, 6,
        3, 5
    };
    
    for (size_t i = 0; i < 12; ++i) {
        auto idx0 = indices[i * 2];
        auto idx1 = indices[i * 2 + 1];

        if ((idx0 == v0_idx && idx1 == v1_idx) ||
            (idx1 == v0_idx && idx0 == v1_idx)) {
            return i;
        }
    }

    return SIZE_MAX;
}

size_t box_shape::get_face_index(size_t v0_idx, size_t v1_idx,
                                 size_t v2_idx, size_t v3_idx) const {
    constexpr size_t indices[] = {
        0, 1, 2, 3,
        4, 5, 6, 7,
        0, 3, 5, 4,
        1, 7, 6, 2,
        0, 4, 7, 1,
        3, 2, 6, 5
    };

    const auto vidx_set = std::unordered_set<size_t>{v0_idx, v1_idx, v2_idx, v3_idx};
    
    for (size_t i = 0; i < 6; ++i) {
        const auto idx_set = std::unordered_set<size_t>{
            indices[i * 4 + 0],
            indices[i * 4 + 1],
            indices[i * 4 + 2],
            indices[i * 4 + 3]
        };

        if (vidx_set == idx_set) {
            return i;
        }
    }

    return SIZE_MAX;
}

}