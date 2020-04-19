#include "edyn/shapes/box_shape.hpp"

namespace edyn {

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

std::tuple<vector3, vector3> box_shape::get_edge(size_t i) const {
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

std::tuple<vector3, vector3, vector3, vector3>
box_shape::get_face(size_t i) const {
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

std::tuple<box_feature, size_t> box_shape::support_feature(const vector3 &dir) const {
    // Faces.
    if (dir.x >= scalar(1) - EDYN_EPSILON) {
        return {BOX_FEATURE_FACE, 0};
    } else if (dir.x <= scalar(-1) + EDYN_EPSILON) {
        return {BOX_FEATURE_FACE, 1};
    } else if (dir.y >= scalar(1) - EDYN_EPSILON) {
        return {BOX_FEATURE_FACE, 2};
    } else if (dir.y <= scalar(-1) + EDYN_EPSILON) {
        return {BOX_FEATURE_FACE, 3};
    } else if (dir.z >= scalar(1) - EDYN_EPSILON) {
        return {BOX_FEATURE_FACE, 4};
    } else if (dir.z <= scalar(-1) + EDYN_EPSILON) {
        return {BOX_FEATURE_FACE, 5};
    }

    // Edges.
    if (std::abs(dir.x) < EDYN_EPSILON) {
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

    if (std::abs(dir.y) < EDYN_EPSILON) {
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

    if (std::abs(dir.z) < EDYN_EPSILON) {
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

}