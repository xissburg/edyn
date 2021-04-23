#include "edyn/shapes/compound_shape.hpp"
#include "edyn/shapes/polyhedron_shape.hpp"
#include "edyn/util/shape_util.hpp"
#include <memory>

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

    for (auto &mesh : meshes) {
        auto center = mesh_centroid(mesh.vertices, mesh.indices, mesh.faces);

        auto polyhedron = polyhedron_shape{};
        polyhedron.mesh = std::make_shared<convex_mesh>();
        polyhedron.mesh->vertices = std::move(mesh.vertices);
        polyhedron.mesh->indices = std::move(mesh.indices);
        polyhedron.mesh->faces = std::move(mesh.faces);

        for (auto &v : polyhedron.mesh->vertices) {
            v -= center;
        }

        polyhedron.mesh->calculate_normals();
        polyhedron.mesh->calculate_edges();

    #ifdef EDYN_DEBUG
        polyhedron.mesh->validate();
    #endif

        add_shape(polyhedron, center, quaternion_identity);
    }

    finish();
}

void compound_shape::finish() {
    auto aabbs = std::vector<AABB>(nodes.size());
    std::transform(nodes.begin(), nodes.end(), aabbs.begin(), 
                    [] (auto &node) { return node.aabb; });
    auto report_leaf = [] (static_tree::tree_node &node, auto ids_begin, auto ids_end) {
        node.id = *ids_begin;
    };
    tree.build(aabbs.begin(), aabbs.end(), report_leaf);
}

}
