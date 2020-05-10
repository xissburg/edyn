#include "edyn/shapes/box_shape.hpp"
#include <map>
#include <unordered_set>

namespace edyn {

AABB box_shape::aabb(const vector3 &pos, const quaternion &orn) const {
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

vector3 box_shape::inertia(scalar mass) const {
    auto extents = half_extents * 2;
    return scalar(1) / scalar(12) * mass * vector3{
        extents.y * extents.y + extents.z * extents.z,
        extents.z * extents.z + extents.x * extents.x,
        extents.x * extents.x + extents.y * extents.y,
    };
}

vector3 box_shape::support_point(const vector3 &dir) const {
    return {
        dir.x > 0 ? half_extents.x : -half_extents.x,
        dir.y > 0 ? half_extents.y : -half_extents.y,
        dir.z > 0 ? half_extents.z : -half_extents.z
    };
}

vector3 box_shape::support_point(const quaternion &orn, const vector3 &dir) const {
    auto local_dir = rotate(conjugate(orn), dir);
    auto pt = support_point(local_dir);
    return rotate(orn, pt);
}

vector3 box_shape::support_point(const vector3 &pos, const quaternion &orn, const vector3 &dir) const {
    return pos + support_point(orn, dir);
}

void box_shape::support_feature(const vector3 &dir, box_feature &feature, 
                                size_t &feature_index, scalar &projection,
                                scalar threshold) const {
    std::array<scalar, 8> projections;
    projection = -EDYN_SCALAR_MAX;

    for (size_t i = 0; i < 8; ++i) {
        auto v = get_vertex(i);
        auto proj = dot(v, dir);
        projections[i] = proj;

        if (proj > projection) {
            projection = proj;
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
        } else { // if (proj2 < proj0 && proj2 < proj1) {
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
    return {
        get_vertex(edge_indices[i * 2 + 0]),
        get_vertex(edge_indices[i * 2 + 1])
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
    return {
        get_vertex(face_indices[i * 4 + 0]),
        get_vertex(face_indices[i * 4 + 1]),
        get_vertex(face_indices[i * 4 + 2]),
        get_vertex(face_indices[i * 4 + 3])
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
    for (size_t i = 0; i < 12; ++i) {
        auto idx0 = edge_indices[i * 2];
        auto idx1 = edge_indices[i * 2 + 1];

        if ((idx0 == v0_idx && idx1 == v1_idx) ||
            (idx1 == v0_idx && idx0 == v1_idx)) {
            return i;
        }
    }

    return SIZE_MAX;
}

size_t box_shape::get_face_index(size_t v0_idx, size_t v1_idx,
                                 size_t v2_idx, size_t v3_idx) const {
    const auto vidx_set = std::unordered_set<size_t>{v0_idx, v1_idx, v2_idx, v3_idx};
    
    for (size_t i = 0; i < 6; ++i) {
        const auto idx_set = std::unordered_set<size_t>{
            face_indices[i * 4 + 0],
            face_indices[i * 4 + 1],
            face_indices[i * 4 + 2],
            face_indices[i * 4 + 3]
        };

        if (vidx_set == idx_set) {
            return i;
        }
    }

    return SIZE_MAX;
}

}