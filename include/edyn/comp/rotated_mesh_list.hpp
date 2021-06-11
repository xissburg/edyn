#ifndef EDYN_COMP_ROTATED_MESH_LIST_HPP
#define EDYN_COMP_ROTATED_MESH_LIST_HPP

#include <memory>
#include <entt/entity/fwd.hpp>
#include <entt/entity/entity.hpp>
#include "edyn/math/quaternion.hpp"
#include "edyn/shapes/convex_mesh.hpp"

namespace edyn {

/**
 * @brief A linked list of rotated meshes.
 * This component is assigned to entities holding a `polyhedron_shape` or a
 * `compound_shape` which contains polyhedrons. When associated with a
 * `polyhedron_shape`, `next` will always be `entt::null`. When associated with
 * a `compound_shape` with more than one polyhedron, new entities are created
 * to hold other rotated meshes.
 */
struct rotated_mesh_list {
    // The original mesh.
    std::shared_ptr<convex_mesh> mesh;

    // The rotated mesh.
    std::unique_ptr<rotated_mesh> rotated;

    // Local orientation to be applied for child nodes of a compound.
    quaternion orientation {quaternion_identity};

    // Entity of next rotated mesh in the linked list.
    entt::entity next {entt::null};
};

}

#endif // EDYN_COMP_ROTATED_MESH_LIST_HPP
