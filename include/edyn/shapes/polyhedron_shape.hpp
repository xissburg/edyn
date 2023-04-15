#ifndef EDYN_SHAPES_POLYHEDRON_HPP
#define EDYN_SHAPES_POLYHEDRON_HPP

#include <memory>
#include <string>
#include "edyn/config/config.h"
#include "edyn/shapes/convex_mesh.hpp"

namespace edyn {

struct polyhedron_shape {
    /**
     * The convex mesh that backs this polyhedron. It's stored in a shared
     * pointer because it's expensive to copy. Shapes are copied into
     * registry operations every time islands are merged and split. This also
     * allows many rigid bodies of the same shape to exist without duplicating
     * data. It is potentially shared with multiple threads (e.g. if it is
     * associated with a static or kinematic rigid body) which makes it unsafe
     * to be modified.
     */
    std::shared_ptr<convex_mesh> mesh;

    /**
     * A rotated mesh which serves as a cache where the rotated vertex positions
     * and face normals are stored after each step of the simulation. This has
     * to be unique for each entity, unlike the `mesh` which can be shared,
     * since this reflects the unique orientation of the rigid body.
     * Since this is modified by the island worker, it's not safe to access it
     * in another thread. The main thread does not need this information by
     * default. If it is needed, a new instance should be created to replace
     * the current in the registry of that thread. This is a weak reference:
     * the owner of the polyhedron instance is responsible for keeping it alive.
     */
    rotated_mesh *rotated {nullptr};

    polyhedron_shape() = default;

    /**
     * @brief Initializes a polyhedron shape with a mesh.
     * @param mesh Shared pointer to a convex mesh.
     */
    polyhedron_shape(std::shared_ptr<convex_mesh> mesh);
};

template<typename Archive>
void serialize(Archive &archive, polyhedron_shape &shape) {
    if constexpr(Archive::is_input::value) {
        shape.mesh = std::make_shared<convex_mesh>();
    } else {
        EDYN_ASSERT(shape.mesh);
    }

    archive(*shape.mesh);
}

}

#endif // EDYN_SHAPES_POLYHEDRON_HPP
