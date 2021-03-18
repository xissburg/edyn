#ifndef EDYN_SHAPES_BOX_SHAPE_HPP
#define EDYN_SHAPES_BOX_SHAPE_HPP

#include "edyn/math/vector3.hpp"
#include "edyn/math/matrix3x3.hpp"
#include "edyn/math/vector2.hpp"
#include "edyn/math/geom.hpp"
#include "edyn/comp/aabb.hpp"
#include <tuple>
#include <array>

namespace edyn {

enum class box_feature {
    vertex,
    edge,
    face
};

struct box_shape {
    vector3 half_extents;

    static constexpr size_t edge_indices[] = {
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

    static constexpr size_t face_indices[] = {
        0, 1, 2, 3,
        4, 5, 6, 7,
        0, 3, 5, 4,
        1, 7, 6, 2,
        0, 4, 7, 1,
        3, 2, 6, 5
    };

    AABB aabb(const vector3 &pos, const quaternion &orn) const;

    vector3 inertia(scalar mass) const;

    /**
     * Get support point in object space.
     * @param dir Direction vector in object space.
     * @return Support point in object space.
     */
    vector3 support_point(const vector3 &dir) const;

    /**
     * Get support point with rotation and no translation.
     * @param orn Orientation.
     * @param dir Direction vector.
     * @return Support point.
     */
    vector3 support_point(const quaternion &orn, const vector3 &dir) const;
    
    /**
     * Get support point in world space.
     * @param pos Position of geometric center.
     * @param orn Orientation.
     * @param dir Direction vector.
     * @return Support point in world space.
     */
    vector3 support_point(const vector3 &pos, const quaternion &orn, const vector3 &dir) const;

    /**
     * Get feature (vertex or edge or face) that's furthest along a direction 
     * in object space.
     * @param dir Direction vector, in object space.
     * @param out_feature The feature type.
     * @param out_feature_index The feature index.
     * @param out_projection The projection of the feature along the direction.
     * @param threshold Parameter that gives extra margin in the feature 
     *        classification.
     */
    void support_feature(const vector3 &dir, box_feature &out_feature, 
                         size_t &out_feature_index, scalar &out_projection,
                         scalar threshold) const;

    /**
     * Get feature (vertex or edge or face) that's furthest along a direction 
     * in world space.
     * @param pos Position of geometric center.
     * @param orn Orientation.
     * @param axis_pos A point in the line used as axis.
     * @param axis_dir Direction vector, in world space.
     * @param out_feature The feature type.
     * @param out_feature_index The feature index.
     * @param out_projection The projection of the feature along the direction.
     * @param threshold Parameter that gives extra margin in the feature 
     *        classification.
     */
    void support_feature(const vector3 &pos, const quaternion &orn, 
                         const vector3 &axis_pos, const vector3 &axis_dir,
                         box_feature &out_feature, size_t &out_feature_index,
                         scalar &out_projection,
                         scalar threshold) const;

    /**
     * Get vertex position in object space.
     * @param i The vertex index in [0, 8).
     * @return Vertex position in object space.
     */
    vector3 get_vertex(size_t i) const;

    /**
     * Get vertex position in world space.
     * @param i The vertex index in [0, 8).
     * @param pos Position of geometric center.
     * @param orn Orientation.
     * @return Vertex position in world space.
     */
    vector3 get_vertex(size_t i, const vector3 &pos, const quaternion &orn) const;

    /**
     * Get vertex positions of an edge in object space.
     * @param i Edge index in [0, 12).
     * @return Position of the two vertices in object space.
     */
    std::array<vector3, 2> get_edge(size_t i) const;

    /**
     * Get vertex positions of an edge in world space.
     * @param i Edge index in [0, 12).
     * @param pos Position of geometric center.
     * @param orn Orientation.
     * @return Position of the two vertices in world space.
     */
    std::array<vector3, 2> get_edge(size_t i, const vector3 &pos, const quaternion &orn) const;

    /**
     * Get the vertex positions of a face in object space.
     * @param i Face index in [0, 6).
     * @return Position of the four vertices in object space.
     */
    std::array<vector3, 4> get_face(size_t i) const;

    /**
     * Get the vertex positions of a face in world space.
     * @param i Face index in [0, 6).
     * @param pos Position of geometric center.
     * @param orn Orientation.
     * @return Position of the four vertices in world space.
     */
    std::array<vector3, 4> get_face(size_t i, const vector3 &pos, const quaternion &orn) const;

    /**
     * Get face normal in object space.
     * @param i Face index in [0, 6).
     * @return Face normal in object space.
     */
    vector3 get_face_normal(size_t i) const;

    /**
     * Get face normal in world space.
     * @param i Face index in [0, 6).
     * @param orn Orientation.
     * @return Face normal in world space.
     */
    vector3 get_face_normal(size_t i, const quaternion &orn) const;

    /**
     * Get point at center of the i-th face in world space.
     * @param i Face index in [0, 6).
     * @param pos Position of geometric center of box.
     * @param orn Orientation of the box.
     * @return Point at center of the i-th face in world space.
     */
    vector3 get_face_center(size_t i, const vector3 &pos, const quaternion &orn) const;

    /**
     * Get a basis representing the tangent space of the i-th face where the x
     * and z axes (i.e. columns 0 and 2 in the matrix) are tangent to the face
     * and the y axis (i.e. column 1 in the matrix) is orthogonal to the face,
     * pointing outside the box.
     * @param i Face index in [0, 6).
     * @param orn Orientation of the box.
     * @return Matrix representing a tangent space basis on the i-th face.
     */
    matrix3x3 get_face_basis(size_t i, const quaternion &orn) const;

    /**
     * Get half of the extent of a rectangular face.
     * @param i Face index in [0, 6).
     * @return Half of the bidimensional extent of the rectangular face, 
     *         according to the basis given by `get_face_basis`.
     */
    vector2 get_face_half_extents(size_t i) const;

    /**
     * Get edge index from vertex indices.
     * @param v0_idx Index of first vertex.
     * @param v1_idx Index of second vertex.
     * @return Edge index.
     * @remarks Order does not matter.
     */
    size_t get_edge_index(size_t v0_idx, size_t v1_idx) const;

    /**
     * Get face index whose normal best aligns with the given direction.
     * @param dir Direction vector.
     * @return Face index.
     */
    size_t support_face_index(const vector3 &dir) const;

    size_t get_vertex_index_from_face(size_t face_idx, size_t face_vertex_idx) const;
};

constexpr size_t get_box_num_features(box_feature feature) {
    switch (feature) {
    case box_feature::face:
        return 6;
    case box_feature::edge:
        return 12;
    case box_feature::vertex:
        return 8;
    }
}

constexpr size_t get_box_feature_num_vertices(box_feature feature) {
    switch (feature) {
    case box_feature::face:
        return 4;
    case box_feature::edge:
        return 2;
    case box_feature::vertex:
        return 1;
    }
}

}

#endif // EDYN_SHAPES_BOX_SHAPE_HPP
