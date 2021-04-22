#ifndef EDYN_SHAPES_COMPOUND_SHAPE_HPP
#define EDYN_SHAPES_COMPOUND_SHAPE_HPP

#include <vector>
#include <variant>
#include <algorithm>
#include "edyn/shapes/sphere_shape.hpp"
#include "edyn/shapes/cylinder_shape.hpp"
#include "edyn/shapes/capsule_shape.hpp"
#include "edyn/shapes/box_shape.hpp"
#include "edyn/shapes/polyhedron_shape.hpp"
#include "edyn/comp/aabb.hpp"
#include "edyn/collision/static_tree.hpp"
#include "edyn/util/aabb_util.hpp"

namespace edyn {

struct compound_shape {
    using shapes_variant_t = std::variant<
        sphere_shape, 
        cylinder_shape,
        capsule_shape,
        box_shape,
        polyhedron_shape
    >;

    struct shape_node {
        shapes_variant_t var;
        vector3 position;
        quaternion orientation;
        AABB aabb;
    };

    template<typename T>
    void add_shape(const T &sh, const vector3 &pos, const quaternion &orn);

    void finish();

    template<typename Func>
    void visit(const AABB &aabb, Func func) const;

    std::vector<shape_node> nodes;
    static_tree tree;

    compound_shape() = default;
    compound_shape(const std::string &path_to_obj);
};

template<typename T>
void compound_shape::add_shape(const T &sh, const vector3 &pos, const quaternion &orn) {
    EDYN_ASSERT(tree.empty()); // Assure tree wasn't built yet.
    auto node = shape_node{};
    node.var = {sh};
    node.position = pos;
    node.orientation = orn;
    node.aabb = shape_aabb(sh, pos, orn);
    nodes.push_back(node);
}

template<typename Func>
void compound_shape::visit(const AABB &aabb, Func func) const {
    EDYN_ASSERT(!tree.empty());
    
    tree.visit(aabb, [&] (auto index) {
        auto &node = nodes[index];
        std::visit([&] (auto &&sh) {
            func(sh, node.position, node.orientation);
        }, node.var);
    });
}

}

#endif // EDYN_SHAPES_COMPOUND_SHAPE_HPP
