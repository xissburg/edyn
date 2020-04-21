#include "edyn/shapes/box_shape.hpp"

namespace edyn {

void box_shape::support_feature(const vector3 &dir, box_feature &feature, 
                                uint8_t &feature_index, scalar &projection) const {
    scalar threshold = 0.0002;

    // Faces.
    if (dir.x >= scalar(1) - threshold) {
        feature = BOX_FEATURE_FACE;
        feature_index = 0;
        projection = half_extents[0];
        return;
    } 
    
    if (dir.x <= scalar(-1) + threshold) {
        feature = BOX_FEATURE_FACE;
        feature_index = 1;
        projection = half_extents[0];
        return;
    }
    
    if (dir.y >= scalar(1) - threshold) {
        feature = BOX_FEATURE_FACE;
        feature_index = 2;
        projection = half_extents[1];
        return;
    }
    
    if (dir.y <= scalar(-1) + threshold) {
        feature = BOX_FEATURE_FACE;
        feature_index = 3;
        projection = half_extents[1];
        return;
    } 
    
    if (dir.z >= scalar(1) - threshold) {
        feature = BOX_FEATURE_FACE;
        feature_index = 4;
        projection = half_extents[2];
        return;
    } 
    
    if (dir.z <= scalar(-1) + threshold) {
        feature = BOX_FEATURE_FACE;
        feature_index = 5;
        projection = half_extents[2];
        return;
    }

    // Edges.
    if (std::abs(dir.x) < threshold) {
        if (dir.y > 0) {
            if (dir.z > 0) {
                feature = BOX_FEATURE_EDGE;
                feature_index = 8;
            } else {
                feature = BOX_FEATURE_EDGE;
                feature_index = 11;
            }
        } else {
            if (dir.z > 0) {
                feature = BOX_FEATURE_EDGE;
                feature_index = 9;
            } else {
                feature = BOX_FEATURE_EDGE;
                feature_index = 10;
            }
        }

        projection = dot(dir, get_edge(feature_index)[0]);
        return;
    }

    if (std::abs(dir.y) < threshold) {
        if (dir.x > 0) {
            if (dir.z > 0) {
                feature = BOX_FEATURE_EDGE;
                feature_index = 0;
            } else {
                feature = BOX_FEATURE_EDGE;
                feature_index = 2;
            }
        } else {
            if (dir.z > 0) {
                feature = BOX_FEATURE_EDGE;
                feature_index = 7;
            } else {
                feature = BOX_FEATURE_EDGE;
                feature_index = 5;
            }
        }

        projection = dot(dir, get_edge(feature_index)[0]);
        return;
    }

    if (std::abs(dir.z) < threshold) {
        if (dir.x > 0) {
            if (dir.y > 0) {
                feature = BOX_FEATURE_EDGE;
                feature_index = 3;
            } else {
                feature = BOX_FEATURE_EDGE;
                feature_index = 1;
            }
        } else {
            if (dir.y > 0) {
                feature = BOX_FEATURE_EDGE;
                feature_index = 4;
            } else {
                feature = BOX_FEATURE_EDGE;
                feature_index = 6;
            }
        }

        projection = dot(dir, get_edge(feature_index)[0]);
        return;
    }

    // Vertices.
    if (dir.x > 0) {
        if (dir.y > 0) {
            if (dir.z > 0) {
                feature = BOX_FEATURE_VERTEX;
                feature_index = 0;
            } else {
                feature = BOX_FEATURE_VERTEX;
                feature_index = 3;
            }
        } else {
            if (dir.z > 0) {
                feature = BOX_FEATURE_VERTEX;
                feature_index = 1;
            } else {
                feature = BOX_FEATURE_VERTEX;
                feature_index = 2;
            }
        }
    } else {
        if (dir.y > 0) {
            if (dir.z > 0) {
                feature = BOX_FEATURE_VERTEX;
                feature_index = 4;
            } else {
                feature = BOX_FEATURE_VERTEX;
                feature_index = 5;
            }
        } else {
            if (dir.z > 0) {
                feature = BOX_FEATURE_VERTEX;
                feature_index = 7;
            } else {
                feature = BOX_FEATURE_VERTEX;
                feature_index = 6;
            }
        }
    }

    projection = dot(dir, get_vertex(feature_index));
}

void box_shape::support_feature(const vector3 &pos, const quaternion &orn, 
                                const vector3 &axis_pos, const vector3 &axis_dir,
                                box_feature &feature, uint8_t &feature_index,
                                scalar &projection) const {
    auto local_dir = rotate(conjugate(orn), axis_dir);
    support_feature(local_dir, feature, feature_index, projection);
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

}