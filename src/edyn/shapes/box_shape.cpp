#include "edyn/shapes/box_shape.hpp"
#include "edyn/math/matrix3x3.hpp"
#include "edyn/util/shape_util.hpp"
#include "edyn/util/moment_of_inertia.hpp"
#include <cstdint>

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
    return moment_of_inertia_solid_box(mass, half_extents * 2);
}

vector3 box_shape::support_point(const vector3 &dir) const {
    return support_point_box(half_extents, dir);
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
    // Find face that's the closest to being a support face for `dir` and then
    // check if an edge or a vertex of this face would be the support feature
    // instead by calculating the highest projection onto dir and which other
    // vertices have a projection that fall within the `threshold`.
    auto candidate_face = support_face_index(dir);
    std::array<scalar, 4> projections;
    projection = -EDYN_SCALAR_MAX;
    std::array<size_t, 4> vertex_indices;

    // Vertex index on face [0, 4) of vertices within threshold. There's always
    // at least one (i.e. the one furthest along `dir`).
    std::array<size_t, 4> indices; 
    size_t count = 1;
    size_t max_proj_idx;

    for (size_t i = 0; i < projections.size(); ++i) {
        auto vertex_idx = get_vertex_index_from_face(candidate_face, i);
        vertex_indices[i] = vertex_idx;
        auto v = get_vertex(vertex_idx);
        auto proj = dot(v, dir);
        projections[i] = proj;

        if (proj > projection) {
            projection = proj;
            indices[0] = i;
            max_proj_idx = i;
        }
    }

    for (size_t i = 0; i < projections.size(); ++i) {
        if (i != max_proj_idx && projections[i] > projection - threshold) {
            indices[count++] = i;
        }
    }

    if (count == 1) {
        feature = box_feature::vertex;
        feature_index = vertex_indices[indices[0]];
    } else if (count == 2) {
        feature = box_feature::edge;
        feature_index = get_edge_index(vertex_indices[indices[0]], 
                                       vertex_indices[indices[1]]);
    } else if (count == 3) {
        feature = box_feature::edge;
        // Select 2 points among the 3 with the higher projections.
        auto proj0 = projections[indices[0]];
        auto proj1 = projections[indices[1]];
        auto proj2 = projections[indices[2]];

        if (proj0 <= proj1 && proj0 <= proj2) {
            feature_index = get_edge_index(vertex_indices[indices[1]], 
                                           vertex_indices[indices[2]]);
        } else if (proj1 <= proj0 && proj1 <= proj2) {
            feature_index = get_edge_index(vertex_indices[indices[0]], 
                                           vertex_indices[indices[2]]);
        } else { // if (proj2 <= proj0 && proj2 <= proj1) {
            feature_index = get_edge_index(vertex_indices[indices[0]], 
                                           vertex_indices[indices[1]]);
        }
    } else {
        feature = box_feature::face;
        feature_index = candidate_face;
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
    EDYN_ASSERT(i < get_box_num_features(box_feature::vertex));
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
    EDYN_ASSERT(i < get_box_num_features(box_feature::edge));
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

static
vector3 get_face_tangent(size_t i) {
    EDYN_ASSERT(i < 6);
    constexpr vector3 normals[] = {
        { 0,  1,  0},
        { 0, -1,  0},
        { 0,  0,  1},
        { 0,  0, -1},
        { 1,  0,  0},
        {-1,  0,  0},
    };
    return normals[i];
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

vector3 box_shape::get_face_center(size_t i, const vector3 &pos, const quaternion &orn) const {
    auto n = get_face_normal(i, orn);
    auto e = half_extents[i / 2];
    return pos + n * e;
}

matrix3x3 box_shape::get_face_basis(size_t i, const quaternion &orn) const {
    auto y = get_face_normal(i);
    auto x = get_face_tangent(i);
    auto z = cross(x, y);
    return matrix3x3_columns(rotate(orn, x), rotate(orn, y), rotate(orn, z));
}

vector2 box_shape::get_face_half_extents(size_t i) const {
    EDYN_ASSERT(i < 6);
    if (i == 0 || i == 1) {
        return vector2{half_extents.y, half_extents.z};
    } else if (i == 2 || i == 3) {
        return vector2{half_extents.z, half_extents.x};
    }
    return vector2{half_extents.x, half_extents.y};
}

size_t box_shape::get_edge_index(size_t v0_idx, size_t v1_idx) const {    
    for (size_t i = 0; i < get_box_num_features(box_feature::edge); ++i) {
        auto idx0 = edge_indices[i * 2];
        auto idx1 = edge_indices[i * 2 + 1];

        if ((idx0 == v0_idx && idx1 == v1_idx) ||
            (idx1 == v0_idx && idx0 == v1_idx)) {
            return i;
        }
    }

    EDYN_ASSERT(false);

    return SIZE_MAX;
}

size_t box_shape::support_face_index(const vector3 &dir) const {
    auto max_idx = max_index_abs(dir);
        
    if (dir[max_idx] < 0) {
        return max_idx * 2 + 1;
    } else {
        return max_idx * 2;
    }
}

size_t box_shape::get_vertex_index_from_face(size_t face_idx, size_t face_vertex_idx) const {
    return face_indices[face_idx * 4 + face_vertex_idx];
}

}
