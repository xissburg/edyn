#include "edyn/shapes/box_shape.hpp"
#include "edyn/math/matrix3x3.hpp"
#include "edyn/math/quaternion.hpp"
#include "edyn/util/shape_util.hpp"
#include "edyn/math/vector2_3_util.hpp"
#include <cstdint>

namespace edyn {

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

scalar box_shape::support_projection(const vector3 &pos, const quaternion &orn, const vector3 &dir) const {
    auto local_dir = rotate(conjugate(orn), dir);
    auto pt = support_point(local_dir);
    return dot(pos, dir) + dot(pt, local_dir);
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
    auto count = size_t{1};
    auto max_proj_idx = size_t{};

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

void box_shape::support_feature(const vector3 &pos, const quaternion &orn,
                                const vector3 &axis_dir, box_feature &feature,
                                size_t &feature_index, scalar threshold) const {
    auto local_dir = rotate(conjugate(orn), axis_dir);
    scalar projection;
    support_feature(local_dir, feature, feature_index, projection, threshold);
}

vector3 box_shape::get_vertex(size_t vertex_idx) const {
    EDYN_ASSERT(vertex_idx < get_box_num_features(box_feature::vertex));
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
    return half_extents * multipliers[vertex_idx];
}

vector3 box_shape::get_vertex(size_t vertex_idx, const vector3 &pos, const quaternion &orn) const {
    return to_world_space(get_vertex(vertex_idx), pos, orn);
}

std::array<vector3, 2> box_shape::get_edge(size_t edge_idx) const {
    EDYN_ASSERT(edge_idx < get_box_num_features(box_feature::edge));
    return {
        get_vertex(edge_indices[edge_idx * 2 + 0]),
        get_vertex(edge_indices[edge_idx * 2 + 1])
    };
}

std::array<vector3, 2> box_shape::get_edge(size_t edge_idx, const vector3 &pos, const quaternion &orn) const {
    auto edge_vertices = get_edge(edge_idx);
    return {
        to_world_space(edge_vertices[0], pos, orn),
        to_world_space(edge_vertices[1], pos, orn)
    };
}

std::array<vector3, 4> box_shape::get_face(size_t face_idx) const {
    EDYN_ASSERT(face_idx < 6);
    return {
        get_vertex(face_indices[face_idx * 4 + 0]),
        get_vertex(face_indices[face_idx * 4 + 1]),
        get_vertex(face_indices[face_idx * 4 + 2]),
        get_vertex(face_indices[face_idx * 4 + 3])
    };
}

std::array<vector3, 4> box_shape::get_face(size_t face_idx, const vector3 &pos, const quaternion &orn) const {
    auto face_vertices = get_face(face_idx);
    return {
        to_world_space(face_vertices[0], pos, orn),
        to_world_space(face_vertices[1], pos, orn),
        to_world_space(face_vertices[2], pos, orn),
        to_world_space(face_vertices[3], pos, orn)
    };
}

static
vector3 get_face_tangent(size_t face_idx) {
    EDYN_ASSERT(face_idx < 6);
    // Tangents point to the "side" of a face.
    constexpr vector3 tangents[] = {
        { 0,  0,  1},
        { 0,  0, -1},
        { 1,  0,  0},
        {-1,  0,  0},
        { 0,  1,  0},
        { 0, -1,  0},
    };
    return tangents[face_idx];
}

vector3 box_shape::get_face_normal(size_t face_idx) const {
    EDYN_ASSERT(face_idx < 6);
    constexpr vector3 normals[] = {
        { 1,  0,  0},
        {-1,  0,  0},
        { 0,  1,  0},
        { 0, -1,  0},
        { 0,  0,  1},
        { 0,  0, -1}
    };
    return normals[face_idx];
}

vector3 box_shape::get_face_normal(size_t face_idx, const quaternion &orn) const {
    return rotate(orn, get_face_normal(face_idx));
}

vector3 box_shape::get_face_center(size_t face_idx, const vector3 &pos, const quaternion &orn) const {
    auto n = get_face_normal(face_idx, orn);
    auto e = half_extents[face_idx / 2];
    return pos + n * e;
}

matrix3x3 box_shape::get_face_basis(size_t face_idx, const quaternion &orn) const {
    auto y = get_face_normal(face_idx);
    auto x = get_face_tangent(face_idx);
    auto z = cross(x, y);
    return matrix3x3_columns(rotate(orn, x), rotate(orn, y), rotate(orn, z));
}

vector2 box_shape::get_face_half_extents(size_t face_idx) const {
    EDYN_ASSERT(face_idx < 6);
    // The x component should have the extent on the side on a face according
    // to its basis and the y component should have the vertical extent.
    if (face_idx == 0 || face_idx == 1) {
        return to_vector2_zy(half_extents);
    } else if (face_idx == 2 || face_idx == 3) {
        return to_vector2_xz(half_extents);
    }
    return to_vector2_yx(half_extents);
}

size_t box_shape::get_face_edge_index(size_t face_idx, size_t edge_idx) const {
    EDYN_ASSERT(face_idx < 6);
    EDYN_ASSERT(edge_idx < 4);
    static constexpr size_t indices[] = {
        0, 1, 2, 3,
        4, 5, 6, 7,
        3, 11, 4, 8,
        9, 6, 10, 1,
        8, 7, 9, 0,
        2, 10, 5, 11
    };
    return indices[face_idx * 4 + edge_idx];
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
    EDYN_ASSERT(face_idx < 6);
    EDYN_ASSERT(face_vertex_idx < 4);
    return face_indices[face_idx * 4 + face_vertex_idx];
}

std::pair<box_feature, size_t> box_shape::get_closest_feature_on_face(size_t face_idx, vector3 point, scalar tolerance) const {
    box_feature feature;
    size_t index;
    auto trimmed_half_extents = half_extents - vector3_one * tolerance;

    if (face_idx == 0 || face_idx == 1) { // X face
        auto positive_face = point.x > 0;

        if (point.y > trimmed_half_extents.y) {
            if (point.z > trimmed_half_extents.z) {
                feature = box_feature::vertex;
                index = positive_face ? 0 : 4;
            } else if (point.z < -trimmed_half_extents.z) {
                feature = box_feature::vertex;
                index = positive_face ? 3 : 5;
            } else {
                feature = box_feature::edge;
                index = positive_face ? 3 : 4;
            }
        } else if (point.y < -trimmed_half_extents.y) {
            if (point.z > trimmed_half_extents.z) {
                feature = box_feature::vertex;
                index = positive_face ? 1 : 7;
            } else if (point.z < -trimmed_half_extents.z) {
                feature = box_feature::vertex;
                index = positive_face ? 2 : 6;
            } else {
                feature = box_feature::edge;
                index = positive_face ? 1 : 6;
            }
        } else if (point.z > trimmed_half_extents.z) {
            feature = box_feature::edge;
            index = positive_face ? 0 : 7;
        } else if (point.z < -trimmed_half_extents.z) {
            feature = box_feature::edge;
            index = positive_face ? 2 : 5;
        } else {
            feature = box_feature::face;
            index = face_idx;
        }
    } else if (face_idx == 2 || face_idx == 3) { // Y face
        auto positive_face = point.y > 0;

        if (point.x > trimmed_half_extents.x) {
            if (point.z > trimmed_half_extents.z) {
                feature = box_feature::vertex;
                index = positive_face ? 0 : 1;
            } else if (point.z < -trimmed_half_extents.z) {
                feature = box_feature::vertex;
                index = positive_face ? 3 : 2;
            } else {
                feature = box_feature::edge;
                index = positive_face ? 3 : 1;
            }
        } else if (point.x < -trimmed_half_extents.x) {
            if (point.z > trimmed_half_extents.z) {
                feature = box_feature::vertex;
                index = positive_face ? 4 : 7;
            } else if (point.z < -trimmed_half_extents.z) {
                feature = box_feature::vertex;
                index = positive_face ? 5 : 6;
            } else {
                feature = box_feature::edge;
                index = positive_face ? 4 : 6;
            }
        } else if (point.z > trimmed_half_extents.z) {
            feature = box_feature::edge;
            index = positive_face ? 8 : 9;
        } else if (point.z < -trimmed_half_extents.z) {
            feature = box_feature::edge;
            index = positive_face ? 11 : 10;
        } else {
            feature = box_feature::face;
            index = face_idx;
        }
    } else if (face_idx == 4 || face_idx == 5) { // Z face
        auto positive_face = point.z > 0;

        if (point.x > trimmed_half_extents.x) {
            if (point.y > trimmed_half_extents.y) {
                feature = box_feature::vertex;
                index = positive_face ? 0 : 3;
            } else if (point.y < -trimmed_half_extents.y) {
                feature = box_feature::vertex;
                index = positive_face ? 1 : 2;
            } else {
                feature = box_feature::edge;
                index = positive_face ? 0 : 2;
            }
        } else if (point.x < -trimmed_half_extents.x) {
            if (point.y > trimmed_half_extents.y) {
                feature = box_feature::vertex;
                index = positive_face ? 4 : 5;
            } else if (point.y < -trimmed_half_extents.y) {
                feature = box_feature::vertex;
                index = positive_face ? 7 : 6;
            } else {
                feature = box_feature::edge;
                index = positive_face ? 7 : 5;
            }
        } else if (point.y > trimmed_half_extents.y) {
            feature = box_feature::edge;
            index = positive_face ? 8 : 11;
        } else if (point.y < -trimmed_half_extents.y) {
            feature = box_feature::edge;
            index = positive_face ? 9 : 10;
        } else {
            feature = box_feature::face;
            index = face_idx;
        }
    }

    return {feature, index};
}

std::array<size_t, 2> box_shape::get_edge_face_indices(size_t edge_idx) const {
    EDYN_ASSERT(edge_idx < get_box_num_features(box_feature::edge));
    static constexpr size_t indices[] = {
        0, 4,
        0, 3,
        0, 5,
        0, 2,
        1, 2,
        1, 5,
        1, 3,
        1, 4,
        4, 2,
        3, 4,
        5, 3,
        2, 5
    };
    return {indices[edge_idx * 2], indices[edge_idx * 2 + 1]};
}

std::array<vector3, 2> box_shape::get_edge_face_normals(size_t edge_idx) const {
    auto face_idx = get_edge_face_indices(edge_idx);
    return {get_face_normal(face_idx[0]), get_face_normal(face_idx[1])};
}

std::array<vector3, 2> box_shape::get_edge_face_normals(size_t edge_idx, const quaternion &orn) const {
    auto normals = get_edge_face_normals(edge_idx);
    return {rotate(orn, normals[0]), rotate(orn, normals[1])};
}

}
