#ifndef EDYN_UTIL_SHAPE_IO_HPP
#define EDYN_UTIL_SHAPE_IO_HPP

#include "edyn/math/quaternion.hpp"
#include "edyn/math/vector3.hpp"
#include "edyn/shapes/compound_shape.hpp"
#include "edyn/shapes/polyhedron_shape.hpp"
#include <vector>
#include <cstdint>
#include <string>

namespace edyn {

struct obj_mesh {
    std::string name;
    std::vector<vector3> vertices;
    std::vector<vector3> colors;
    std::vector<uint32_t> indices;
    std::vector<uint32_t> faces;
};

/**
 * @brief Loads meshes from a *.obj file.
 * Scale, rotation and translation are applied to all vertices in this order.
 * @param path Path to file.
 * @param meshes Array to be filled with meshes.
 * @param pos Position offset to add to vertices.
 * @param orn Orientation to rotate vertices.
 * @param scale Scaling to be applied to all vertices.
 * @return Success or failure.
 */
bool load_meshes_from_obj(const std::string &path,
                          std::vector<obj_mesh> &meshes,
                          vector3 pos = vector3_zero,
                          quaternion orn = quaternion_identity,
                          vector3 scale = vector3_one);

void load_meshes_from_obj(std::stringstream &ss,
                          std::vector<obj_mesh> &meshes,
                          vector3 pos = vector3_zero,
                          quaternion orn = quaternion_identity,
                          vector3 scale = vector3_one);

/**
 * @brief Loads a triangle mesh from a *.obj file which must've been
 * triangulated during export.
 * @param path Path to file.
 * @param vertices Array to be filled with vertices.
 * @param indices Array to be filled with indices for each triangle.
 * @param colors Array to be filled with vertex colors.
 * @param pos Position offset to add to vertices.
 * @param orn Orientation to rotate vertices.
 * @param scale Scaling to be applied to all vertices.
 * @return Success or failure.
 */
bool load_tri_mesh_from_obj(const std::string &path,
                            std::vector<vector3> &vertices,
                            std::vector<uint32_t> &indices,
                            std::vector<vector3> *colors = nullptr,
                            vector3 pos = vector3_zero,
                            quaternion orn = quaternion_identity,
                            vector3 scale = vector3_one);

void load_tri_mesh_from_obj(std::stringstream &ss,
                            std::vector<vector3> &vertices,
                            std::vector<uint32_t> &indices,
                            std::vector<vector3> *colors = nullptr,
                            vector3 pos = vector3_zero,
                            quaternion orn = quaternion_identity,
                            vector3 scale = vector3_one);

struct polyhedron_with_center {
    polyhedron_shape shape;
    vector3 center;
};

/**
 * @brief Load convex polyhedrons from an obj file.
 * @remark The transform is applied in this order: scale, rotation,
 * translation.
 * @param path_to_obj File path.
 * @param pos Offset to apply to all vertex positions.
 * @param orn Orientation to rotate all vertices.
 * @param scale Scaling to apply to all vertices.
 * @return List of polyhedrons with their respective centroid.
 */
std::vector<polyhedron_with_center> load_convex_polyhedrons_from_obj(
    const std::string &path_to_obj,
    const vector3 &pos = vector3_zero,
    const quaternion &orn = quaternion_identity,
    const vector3 &scale = vector3_one);

std::vector<polyhedron_with_center> load_convex_polyhedrons_from_obj(
    std::stringstream &ss,
    const vector3 &pos = vector3_zero,
    const quaternion &orn = quaternion_identity,
    const vector3 &scale = vector3_one);

/**
 * @brief Loads a compound shape from an obj file. All child shapes are created
 * as polyhedrons.
 * @remark The transform is applied in this order: scale, rotation,
 * translation.
 * @param path_to_obj File path.
 * @param pos Offset to apply to all vertex positions.
 * @param orn Orientation to rotate all vertices.
 * @param scale Scaling to apply to all vertices.
 * @return Compound shape.
 */
compound_shape load_compound_shape_from_obj(
    const std::string &path_to_obj,
    const vector3 &pos = vector3_zero,
    const quaternion &orn = quaternion_identity,
    const vector3 &scale = vector3_one);

compound_shape load_compound_shape_from_obj(
    std::stringstream &ss,
    const vector3 &pos = vector3_zero,
    const quaternion &orn = quaternion_identity,
    const vector3 &scale = vector3_one);

}

#endif // EDYN_UTIL_SHAPE_IO_HPP
