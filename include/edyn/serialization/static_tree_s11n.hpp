#ifndef EDYN_SERIALIZATION_STATIC_TREE_S11N_HPP
#define EDYN_SERIALIZATION_STATIC_TREE_S11N_HPP

#include "edyn/collision/static_tree.hpp"
#include "edyn/serialization/std_s11n.hpp"

namespace edyn {

template<typename Archive>
void serialize(Archive &archive, static_tree::tree_node &node) {
    archive(node.aabb.min);
    archive(node.aabb.max);
    archive(node.child1);
    archive(node.child2);
}

template<typename Archive>
void serialize(Archive &archive, static_tree &tree) {
    archive(tree.m_nodes);
}

inline
size_t serialization_sizeof(const static_tree::tree_node &node) {
    return
        sizeof(node.aabb.min) +
        sizeof(node.aabb.max) +
        sizeof(node.child1) +
        sizeof(node.child2);
}

inline
size_t serialization_sizeof(const static_tree &tree) {
    return serialization_sizeof(tree.m_nodes);
}

}

#endif // EDYN_SERIALIZATION_STATIC_TREE_S11N_HPP
