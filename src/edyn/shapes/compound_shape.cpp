#include "edyn/shapes/compound_shape.hpp"

namespace edyn {

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
