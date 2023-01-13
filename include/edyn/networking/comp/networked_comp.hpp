#ifndef EDYN_NETWORKING_NETWORKED_COMP_HPP
#define EDYN_NETWORKING_NETWORKED_COMP_HPP

#include "edyn/comp/child_list.hpp"
#include "edyn/comp/gravity.hpp"
#include "edyn/comp/linvel.hpp"
#include "edyn/comp/angvel.hpp"
#include "edyn/comp/mass.hpp"
#include "edyn/comp/inertia.hpp"
#include "edyn/comp/position.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/comp/center_of_mass.hpp"
#include "edyn/constraints/constraint.hpp"
#include "edyn/comp/tag.hpp"
#include "edyn/comp/shape_index.hpp"
#include "edyn/comp/material.hpp"
#include "edyn/comp/island.hpp"
#include "edyn/comp/collision_filter.hpp"
#include "edyn/comp/collision_exclusion.hpp"
#include "edyn/comp/roll_direction.hpp"
#include "edyn/networking/comp/action_history.hpp"
#include "edyn/networking/comp/asset_ref.hpp"
#include "edyn/shapes/sphere_shape.hpp"
#include "edyn/shapes/cylinder_shape.hpp"
#include "edyn/shapes/capsule_shape.hpp"
#include "edyn/shapes/box_shape.hpp"
#include "edyn/shapes/polyhedron_shape.hpp"
#include "edyn/shapes/compound_shape.hpp"
#include "edyn/shapes/plane_shape.hpp"
#include "edyn/networking/comp/entity_owner.hpp"
#include "edyn/constraints/null_constraint.hpp"

namespace edyn {

/**
 * @brief Tuple of components which are shared over the network.
 */
static const auto networked_components = std::tuple_cat(std::tuple<
    collision_filter,
    collision_exclusion,
    inertia,
    gravity,
    angvel,
    linvel,
    mass,
    material,
    position,
    orientation,
    center_of_mass,
    dynamic_tag,
    kinematic_tag,
    static_tag,
    procedural_tag,
    sleeping_disabled_tag,
    disabled_tag,
    external_tag,
    rigidbody_tag,
    constraint_tag,
    rolling_tag,
    roll_direction,
    null_constraint,
    gravity_constraint,
    point_constraint,
    distance_constraint,
    soft_distance_constraint,
    hinge_constraint,
    generic_constraint,
    cone_constraint,
    cvjoint_constraint,
    sphere_shape,
    cylinder_shape,
    capsule_shape,
    box_shape,
    polyhedron_shape,
    compound_shape,
    plane_shape,
    shape_index,
    entity_owner,
    action_history,
    asset_ref,
    child_list,
    parent_comp
>{});

using networked_components_t = std::decay_t<decltype(networked_components)>;

}

#endif // EDYN_NETWORKING_NETWORKED_COMP_HPP
