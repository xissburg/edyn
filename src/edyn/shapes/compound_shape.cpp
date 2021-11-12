#include "edyn/shapes/compound_shape.hpp"
#include "edyn/shapes/polyhedron_shape.hpp"
#include "edyn/util/shape_util.hpp"
#include "edyn/util/aabb_util.hpp"

namespace edyn {

compound_shape::compound_shape(const std::string &path_to_obj,
                               const vector3 &pos,
                               const quaternion &orn,
                               const vector3 &scale) {
    auto meshes = std::vector<obj_mesh>{};

    if (!load_meshes_from_obj(path_to_obj, meshes, pos, orn, scale)) {
        return;
    }

    EDYN_ASSERT(!meshes.empty());

    // Create a polyhedron shape for each mesh.
    for (auto &mesh : meshes) {
        auto center = mesh_centroid(mesh.vertices, mesh.indices, mesh.faces);

        // Make all vertices to be positioned with respect to the centroid.
        // This is important for correct moment of inertia calculation using
        // the parallel axis theorem, which requires the moment of inertia
        // about the center of mass as input.
        for (auto &v : mesh.vertices) {
            v -= center;
        }

        auto polyhedron = polyhedron_shape{};
        polyhedron.mesh = std::make_shared<convex_mesh>();
        polyhedron.mesh->vertices = std::move(mesh.vertices);
        polyhedron.mesh->indices = std::move(mesh.indices);
        polyhedron.mesh->faces = std::move(mesh.faces);
        polyhedron.mesh->update_calculated_properties();

    #ifdef EDYN_DEBUG
        polyhedron.mesh->validate();
    #endif

        add_shape(polyhedron, center, quaternion_identity);
    }

    finish();
}

void compound_shape::finish() {
    EDYN_ASSERT(!nodes.empty());

    // Calculate node aabbs.
    auto aabbs = std::vector<AABB>{};
    aabbs.reserve(nodes.size());

    for (auto &node : nodes) {
        std::visit([&node] (auto &&shape) {
            node.aabb = shape_aabb(shape, node.position, node.orientation);
        }, node.shape_var);
        aabbs.push_back(node.aabb);
    }

    auto report_leaf = [] (static_tree::tree_node &node, auto ids_begin, auto ids_end) {
        node.id = *ids_begin;
    };
    tree.build(aabbs.begin(), aabbs.end(), report_leaf);
}

}
