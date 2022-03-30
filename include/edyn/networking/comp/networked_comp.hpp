#ifndef EDYN_NETWORKING_NETWORKED_COMP_HPP
#define EDYN_NETWORKING_NETWORKED_COMP_HPP

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
#include "edyn/comp/continuous.hpp"
#include "edyn/comp/roll_direction.hpp"
#include "edyn/shapes/sphere_shape.hpp"
#include "edyn/shapes/cylinder_shape.hpp"
#include "edyn/shapes/capsule_shape.hpp"
#include "edyn/shapes/box_shape.hpp"
#include "edyn/shapes/polyhedron_shape.hpp"
#include "edyn/shapes/compound_shape.hpp"
#include "edyn/shapes/plane_shape.hpp"
#include "edyn/networking/comp/entity_owner.hpp"

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
    continuous,
    center_of_mass,
    dynamic_tag,
    kinematic_tag,
    static_tag,
    procedural_tag,
    sleeping_disabled_tag,
    disabled_tag,
    continuous_contacts_tag,
    external_tag,
    shape_index,
    rigidbody_tag,
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
    entity_owner
>{});

using networked_components_t = std::decay_t<decltype(networked_components)>;

}

#endif // EDYN_NETWORKING_NETWORKED_COMP_HPP
