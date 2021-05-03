#ifndef EDYN_COMP_SHAPE_HPP
#define EDYN_COMP_SHAPE_HPP

#include "edyn/shapes/plane_shape.hpp"
#include "edyn/shapes/sphere_shape.hpp"
#include "edyn/shapes/cylinder_shape.hpp"
#include "edyn/shapes/capsule_shape.hpp"
#include "edyn/shapes/mesh_shape.hpp"
#include "edyn/shapes/box_shape.hpp"
#include "edyn/shapes/polyhedron_shape.hpp"
#include "edyn/shapes/paged_mesh_shape.hpp"
#include "edyn/shapes/compound_shape.hpp"
#include "edyn/util/tuple_util.hpp"
#include "edyn/util/entt_util.hpp"
#include "edyn/util/visit_component.hpp"
#include <entt/fwd.hpp>
#include <utility>

namespace edyn {

// All shape types.
using shapes_tuple_t = std::tuple<
    plane_shape,
    sphere_shape,
    cylinder_shape,
    capsule_shape,
    mesh_shape,
    box_shape,
    polyhedron_shape,
    paged_mesh_shape,
    compound_shape
>;

// Types of shapes that can be transformed.
using non_static_shapes_tuple_t = std::tuple<
    sphere_shape,
    cylinder_shape,
    capsule_shape,
    box_shape,
    polyhedron_shape,
    compound_shape
>;

using shapes_variant_t = tuple_to_variant<shapes_tuple_t>::type;

struct shape_index {
    size_t value;
};

template<typename ShapeType>
constexpr size_t get_shape_index() {
    return index_of<ShapeType>(shapes_tuple_t{});
}

inline auto get_tuple_of_shape_views(entt::registry &registry) {
    return get_tuple_of_views(registry, shapes_tuple_t{});
}

template<typename... Ts>
struct map_to_tuple_of_views;

template<typename... Ts>
struct map_to_tuple_of_views<std::tuple<Ts...>> {
    using type = std::tuple<entt::basic_view<entt::entity, entt::exclude_t<>, Ts>...>;
};

using tuple_of_shape_views_t = map_to_tuple_of_views<shapes_tuple_t>::type;

template<typename VisitorType>
void visit_shape(const shape_index &index, entt::entity entity,
                 const tuple_of_shape_views_t &views_tuple, VisitorType visitor) {
    visit_component(shapes_tuple_t{}, index.value, entity, views_tuple, visitor);
}

/**
 * @brief Visits the shape variant and calculates the the AABB.
 * @param var The shape variant.
 * @param pos Shape's position.
 * @param orn Shape's orientation.
 * @return The AABB.
 */
AABB shape_aabb(const shapes_variant_t &var, const vector3 &pos, const quaternion &orn);

}

#endif // EDYN_COMP_SHAPE_HPP
