#ifndef EDYN_SHAPES_POLYHEDRON_HPP
#define EDYN_SHAPES_POLYHEDRON_HPP

#include <memory>
#include "edyn/shapes/convex_mesh.hpp"

namespace edyn {

struct polyhedron_shape {
    /**
     * The convex mesh that backs this polyhedron. It is potentially shared
     * with multiple threads (e.g. if it is associated with a static or
     * kinematic rigid body) which makes it unsafe to be modified.
     */
    std::shared_ptr<convex_mesh> mesh;

    /**
     * A rotated mesh which serves as a cache where the rotated vertex positions
     * and face normals are stored after each step of the simulation. Since this
     * is modified by the island worker, it's not safe to access it in another
     * thread. The coordinator does not need this information by default. If it
     * is needed, a new instance should be created to replace the current in the
     * registry of that thread.
     */
    std::shared_ptr<rotated_mesh> rotated;

    polyhedron_shape() = default;

    /**
     * @brief Initializes a polyhedron shape with a mesh.
     * @param mesh Shared pointer to a convex mesh.
     */
    polyhedron_shape(std::shared_ptr<convex_mesh> mesh);

    /**
     * @brief Loads a polyhedron shape from an obj file. 
     * The obj file must have a single mesh. If the obj file contains more than 
     * one mesh, use a `compound_shape` instead.
     * @remark The transform is applied in this order: scale, rotation,
     * translation.
     * @param path_to_obj File path.
     * @param pos Offset to apply to all vertex positions.
     * @param orn Orientation to rotate all vertices.
     * @param scale Scaling to apply to all vertices.
     */
    polyhedron_shape(const std::string &path_to_obj,
                     const vector3 &pos = vector3_zero,
                     const quaternion &orn = quaternion_identity,
                     const vector3 &scale = vector3_one);
};

}

#endif // EDYN_SHAPES_POLYHEDRON_HPP
