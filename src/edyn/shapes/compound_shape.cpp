#include "edyn/shapes/compound_shape.hpp"
#include "edyn/shapes/polyhedron_shape.hpp"
#include "edyn/util/shape_util.hpp"
#include "edyn/util/aabb_util.hpp"

namespace edyn {

void compound_shape::finish() {
    EDYN_ASSERT(!nodes.empty());

    // Calculate node aabbs.
    auto aabbs = std::vector<AABB>{};
    aabbs.reserve(nodes.size());

    for (auto &node : nodes) {
        std::visit([&node](auto &&shape) {
            node.aabb = shape_aabb(shape, node.position, node.orientation);
        }, node.shape_var);
        aabbs.push_back(node.aabb);
    }

    auto report_leaf = [](static_tree::tree_node &node, auto ids_begin, auto ids_end) {
        node.id = *ids_begin;
    };
    tree.build(aabbs.begin(), aabbs.end(), report_leaf);
}

}
