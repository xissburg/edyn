#ifndef EDYN_SERIALIZATION_SHAPE_COMPOUND_SHAPE_S11N_HPP
#define EDYN_SERIALIZATION_SHAPE_COMPOUND_SHAPE_S11N_HPP

#include "edyn/shapes/compound_shape.hpp"
namespace edyn {

template<typename Archive>
void serialize(Archive &archive, compound_shape::shape_node &node) {
    archive(node.position);
    archive(node.orientation);
    archive(node.aabb);
    archive(node.shape_var);
}

template<typename Archive>
void serialize(Archive &archive, compound_shape &shape) {
    archive(shape.nodes);
}

}

#endif // EDYN_SERIALIZATION_SHAPE_COMPOUND_SHAPE_S11N_HPP
