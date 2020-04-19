#include "edyn/shapes/box_shape.hpp"

namespace edyn {

std::tuple<box_feature, size_t> box_shape::support_feature(const vector3 &dir) const {
    scalar threshold = 0.0002;

    // Faces.
    if (dir.x >= scalar(1) - threshold) {
        return {BOX_FEATURE_FACE, 0};
    } else if (dir.x <= scalar(-1) + threshold) {
        return {BOX_FEATURE_FACE, 1};
    } else if (dir.y >= scalar(1) - threshold) {
        return {BOX_FEATURE_FACE, 2};
    } else if (dir.y <= scalar(-1) + threshold) {
        return {BOX_FEATURE_FACE, 3};
    } else if (dir.z >= scalar(1) - threshold) {
        return {BOX_FEATURE_FACE, 4};
    } else if (dir.z <= scalar(-1) + threshold) {
        return {BOX_FEATURE_FACE, 5};
    }

    // Edges.
    if (std::abs(dir.x) < threshold) {
        if (dir.y > 0) {
            if (dir.z > 0) {
                return {BOX_FEATURE_EDGE, 8};
            } else {
                return {BOX_FEATURE_EDGE, 11};
            }
        } else {
            if (dir.z > 0) {
                return {BOX_FEATURE_EDGE, 9};
            } else {
                return {BOX_FEATURE_EDGE, 10};
            }
        }
    }

    if (std::abs(dir.y) < threshold) {
        if (dir.x > 0) {
            if (dir.z > 0) {
                return {BOX_FEATURE_EDGE, 0};
            } else {
                return {BOX_FEATURE_EDGE, 2};
            }
        } else {
            if (dir.z > 0) {
                return {BOX_FEATURE_EDGE, 7};
            } else {
                return {BOX_FEATURE_EDGE, 5};
            }
        }
    }

    if (std::abs(dir.z) < threshold) {
        if (dir.x > 0) {
            if (dir.y > 0) {
                return {BOX_FEATURE_EDGE, 3};
            } else {
                return {BOX_FEATURE_EDGE, 1};
            }
        } else {
            if (dir.y > 0) {
                return {BOX_FEATURE_EDGE, 4};
            } else {
                return {BOX_FEATURE_EDGE, 6};
            }
        }
    }

    // Vertices.
    if (dir.x > 0) {
        if (dir.y > 0) {
            if (dir.z > 0) {
                return {BOX_FEATURE_VERTEX, 0};
            } else {
                return {BOX_FEATURE_VERTEX, 3};
            }
        } else {
            if (dir.z > 0) {
                return {BOX_FEATURE_VERTEX, 1};
            } else {
                return {BOX_FEATURE_VERTEX, 2};
            }
        }
    } else {
        if (dir.y > 0) {
            if (dir.z > 0) {
                return {BOX_FEATURE_VERTEX, 4};
            } else {
                return {BOX_FEATURE_VERTEX, 5};
            }
        } else {
            if (dir.z > 0) {
                return {BOX_FEATURE_VERTEX, 7};
            } else {
                return {BOX_FEATURE_VERTEX, 6};
            }
        }
    }
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

}