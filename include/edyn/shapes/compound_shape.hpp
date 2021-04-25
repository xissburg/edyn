#ifndef EDYN_SHAPES_COMPOUND_SHAPE_HPP
#define EDYN_SHAPES_COMPOUND_SHAPE_HPP

#include <vector>
#include <variant>
#include "edyn/shapes/sphere_shape.hpp"
#include "edyn/shapes/cylinder_shape.hpp"
#include "edyn/shapes/capsule_shape.hpp"
#include "edyn/shapes/box_shape.hpp"
#include "edyn/shapes/polyhedron_shape.hpp"
#include "edyn/comp/aabb.hpp"
#include "edyn/collision/static_tree.hpp"
#include "edyn/util/aabb_util.hpp"

namespace edyn {

/**
 * @brief A shape which contains other shapes.
 */
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

    /**
     * @brief Inserts another shape into the compound.
     * @tparam T Shape type. Must be one of the types in `shapes_variant_t`.
     * @param sh The shape to be inserted.
     * @param pos Position of the shape in the compound's object space.
     * @param orn Orientation of the shape in the compound's object space.
     */
    template<typename T>
    void add_shape(const T &sh, const vector3 &pos, const quaternion &orn);

    /**
     * @brief Must be called after all shapes are added to wrap up the compound
     * construction.
     */
    void finish();

    /**
     * @brief Visits the shapes whose AABB intersects the given AABB.
     * @tparam Func Visitor function type.
     * @param aabb AABB in compound's object space to be visited.
     * @param func Function to be called for each shape whose AABB intersects
     * the AABB of interest. Signature: `void(auto &&, shape_node &)`, where
     * the first parameter is the shape.
     */
    template<typename Func>
    void visit(const AABB &aabb, Func func) const;

    compound_shape() = default;

    /**
     * @brief Loads a compound shape from an obj file.
     * @remark The transform is applied in this order: scale, rotation,
     * translation.
     * @param path_to_obj File path.
     * @param pos Offset to apply to all vertex positions.
     * @param orn Orientation to rotate all vertices.
     * @param scale Scaling to apply to all vertices.
     */
    compound_shape(const std::string &path_to_obj,
                   const vector3 &pos = vector3_zero,
                   const quaternion &orn = quaternion_identity,
                   const vector3 &scale = vector3_one);

    std::vector<shape_node> nodes;
    static_tree tree;
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
            func(sh, node);
        }, node.var);
    });
}

}

#endif // EDYN_SHAPES_COMPOUND_SHAPE_HPP
