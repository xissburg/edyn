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
#include <entt/entity/fwd.hpp>

namespace edyn {

// Shapes that can be transformed.
static const auto dynamic_shapes_tuple = std::tuple<
    sphere_shape,
    cylinder_shape,
    capsule_shape,
    box_shape,
    polyhedron_shape,
    compound_shape
>{};

// Shapes which can't be transformed.
static const auto static_shapes_tuple = std::tuple<
    plane_shape,
    mesh_shape,
    paged_mesh_shape
>{};

// Shapes that can roll.
static const auto rolling_shapes_tuple = std::tuple<
    sphere_shape,
    cylinder_shape,
    capsule_shape
>{};

using rolling_shapes_tuple_t = std::decay_t<decltype(rolling_shapes_tuple)>;

// Tuple containing all shape types.
static const auto shapes_tuple = std::tuple_cat(dynamic_shapes_tuple, static_shapes_tuple);

// Variant containing all shape types.
static const auto shapes_variant = tuple_to_variant(shapes_tuple);
using shapes_variant_t = std::decay_t<decltype(shapes_variant)>;

// Variant containing all possible shape feature types.
using shape_feature_t = std::variant<
    box_feature,
    cylinder_feature,
    capsule_feature,
    triangle_feature
>;

/**
 * @brief Returns the index value of a shape type. This is the value that
 * should be stored in a `shape_index` component.
 * @tparam ShapeType One of the shape types in `shapes_tuple`.
 * @return Index of shape type.
 */
template<typename ShapeType>
constexpr auto get_shape_index() {
    return tuple_index_of<ShapeType, shape_index::index_type>(shapes_tuple);
}

/**
 * @brief Creates a tuple of shape views, such as
 * `std::make_tuple(registry.view<sphere_shape>(), registry.view<cylinder_shape>, ...)`.
 * @param registry The source of shapes.
 */
inline auto get_tuple_of_shape_views(entt::registry &registry) {
    return get_tuple_of_views(registry, shapes_tuple);
}

using tuple_of_shape_views_t = map_to_tuple_of_views<std::decay_t<decltype(shapes_tuple)>>::type;

/**
 * @brief Obtains the shape held by `entity` using a shape index and passes
 * it to `visitor`. There must be a shape of the correct type assigned to the
 * given entity in the registry.
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
    visit_component(shapes_tuple, index.value, entity, views_tuple, visitor);
}

/**
 * @brief Obtains the shape held by `entity` using a shape index and passes
 * it to `visitor`. There must be a shape of the correct type associated to
 * the given entity in the registry.
 * @tparam VisitorType Visitor function type.
 * @param registry Source of shapes.
 * @param entity The entity holding the shape.
 * @param visitor Function to be called once with the shape corresponding to
 * the shape `index`.
 */
template<typename VisitorType>
void visit_shape(entt::registry &registry, entt::entity entity, VisitorType visitor) {
    auto &index = registry.get<shape_index>(entity);
    visit_component(shapes_tuple, index.value, entity, registry, visitor);
}

/**
 * @brief Get the restricted rolling direction of a shape, if it has one.
 * @tparam ShapeType A type of shape that can roll.
 * @return A unit vector or the zero vector if the shape can roll in any
 * direction.
 */
template<typename ShapeType, std::enable_if_t<has_type<ShapeType, rolling_shapes_tuple_t>::value, bool> = true>
constexpr vector3 shape_rolling_direction() {
    if constexpr(std::is_same_v<ShapeType, cylinder_shape>) {
        return vector3_x;
    }

    return vector3_zero;
}

}

#endif // EDYN_SHAPES_SHAPES_HPP
