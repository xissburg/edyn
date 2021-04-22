#include "edyn/shapes/compound_shape.hpp"
#include "edyn/shapes/polyhedron_shape.hpp"
#include "edyn/util/shape_util.hpp"
#include <memory>

namespace edyn {

compound_shape::compound_shape(const std::string &path_to_obj) {
    auto meshes = std::vector<obj_mesh>{};

    if (!load_meshes_from_obj(path_to_obj, meshes)) {
        return;
    }

    for (auto &mesh : meshes) {
        auto center = mesh_center_of_mass(mesh.vertices, mesh.indices, mesh.faces);

        auto polyhedron = polyhedron_shape{};
        polyhedron.mesh = std::make_shared<convex_mesh>();

        polyhedron.mesh->vertices = std::move(mesh.vertices);
        polyhedron.mesh->indices = std::move(mesh.indices);
        polyhedron.mesh->faces = std::move(mesh.faces);

        for (auto &v : polyhedron.mesh->vertices) {
            v -= center;
        }

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
