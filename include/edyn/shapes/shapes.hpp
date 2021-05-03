#ifndef EDYN_SHAPES_SHAPES_HPP
#define EDYN_SHAPES_SHAPES_HPP

#include "edyn/shapes/plane_shape.hpp"
#include "edyn/shapes/sphere_shape.hpp"
#include "edyn/shapes/cylinder_shape.hpp"
#include "edyn/shapes/capsule_shape.hpp"
#include "edyn/shapes/mesh_shape.hpp"
#include "edyn/shapes/box_shape.hpp"
#include "edyn/shapes/polyhedron_shape.hpp"
#include "edyn/shapes/paged_mesh_shape.hpp"
#include "edyn/shapes/compound_shape.hpp"
#include "edyn/comp/shape_index.hpp"
#include "edyn/util/tuple_util.hpp"
#include "edyn/util/entt_util.hpp"
#include "edyn/util/visit_component.hpp"
#include <entt/fwd.hpp>

namespace edyn {

// Types of shapes that can be transformed.
using dynamic_shapes_tuple_t = std::tuple<
    sphere_shape,
    cylinder_shape,
    capsule_shape,
    box_shape,
    polyhedron_shape,
    compound_shape
>;

// Types of shapes which can't be transformed.
using static_shapes_tuple_t = std::tuple<
    plane_shape,
    mesh_shape,
    paged_mesh_shape
>;

// Type of tuple containing all shape types.
using shapes_tuple_t = tuple_type_cat<dynamic_shapes_tuple_t, static_shapes_tuple_t>::type;

// Type of variant containing all shape types.
using shapes_variant_t = tuple_to_variant<shapes_tuple_t>::type;

/**
 * @brief Returns the index value of a shape type. This is the value that's
 * stored in a `shape_index` component.
 * @tparam ShapeType One of the shape types in `shapes_tuple_t`.
 * @return Index of shape type.
 */
template<typename ShapeType>
constexpr size_t get_shape_index() {
    return index_of<ShapeType>(shapes_tuple_t{});
}

/**
 * @brief Creates a tuple of shape views, such as
 * `std::make_tuple(registry.view<sphere_shape>(), registry.view<cylinder_shape>, ...)`.
 * @param registry The source of shapes.
 */
inline auto get_tuple_of_shape_views(entt::registry &registry) {
    return get_tuple_of_views(registry, shapes_tuple_t{});
}

using tuple_of_shape_views_t = map_to_tuple_of_views<shapes_tuple_t>::type;

/**
 * @brief Obtains the shape held by `entity` using a shape index and passes
 * it to `visitor`. There must be a shape of the correct type associated to
 * the given entity in the registry.
 * @tparam VisitorType Visitor function type.
 * @param index Shape index, usually obtained from a `shape_index` component.
 * @param entity The entity holding the shape.
 * @param views_tuple Tuple containing a view of each one of all shape types.
 * @param visitor Function to be called once with the shape corresponding to
 * the shape `index`.
 */
template<typename VisitorType>
void visit_shape(const shape_index &index, entt::entity entity,
                 const tuple_of_shape_views_t &views_tuple, VisitorType visitor) {
    visit_component(shapes_tuple_t{}, index.value, entity, views_tuple, visitor);
}

/**
 * @brief Obtains the shape held by `entity` using a shape index and passes
 * it to `visitor`. There must be a shape of the correct type associated to
 * the given entity in the registry.
 * @tparam VisitorType Visitor function type.
 * @param index Shape index, usually obtained from a `shape_index` component.
 * @param entity The entity holding the shape.
 * @param registry Source of shapes.
 * @param visitor Function to be called once with the shape corresponding to
 * the shape `index`.
 */
template<typename VisitorType>
void visit_shape(const shape_index &index, entt::entity entity,
                 entt::registry &registry, VisitorType visitor) {
    visit_component(shapes_tuple_t{}, index.value, entity, registry, visitor);
}

}

#endif // EDYN_SHAPES_SHAPES_HPP
