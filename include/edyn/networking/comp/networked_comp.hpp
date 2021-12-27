#ifndef EDYN_NETWORKING_NETWORKED_COMP_HPP
#define EDYN_NETWORKING_NETWORKED_COMP_HPP

#include "edyn/comp/aabb.hpp"
#include "edyn/comp/gravity.hpp"
#include "edyn/comp/linvel.hpp"
#include "edyn/comp/angvel.hpp"
#include "edyn/comp/mass.hpp"
#include "edyn/comp/inertia.hpp"
#include "edyn/comp/position.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/comp/center_of_mass.hpp"
#include "edyn/comp/present_orientation.hpp"
#include "edyn/comp/present_position.hpp"
#include "edyn/constraints/constraint.hpp"
#include "edyn/constraints/constraint_impulse.hpp"
#include "edyn/comp/tag.hpp"
#include "edyn/comp/shape_index.hpp"
#include "edyn/comp/material.hpp"
#include "edyn/comp/island.hpp"
#include "edyn/comp/collision_filter.hpp"
#include "edyn/comp/collision_exclusion.hpp"
#include "edyn/comp/continuous.hpp"
#include "edyn/shapes/shapes.hpp"
#include "edyn/networking/comp/entity_owner.hpp"

namespace edyn {

static const auto networked_components = std::tuple_cat(shapes_tuple, std::tuple<
    AABB,
    collision_filter,
    collision_exclusion,
    constraint_impulse,
    inertia,
    inertia_inv,
    inertia_world_inv,
    gravity,
    angvel,
    linvel,
    mass,
    mass_inv,
    material,
    position,
    orientation,
    continuous,
    center_of_mass,
    dynamic_tag,
    kinematic_tag,
    static_tag,
    procedural_tag,
    sleeping_tag,
    sleeping_disabled_tag,
    disabled_tag,
    continuous_contacts_tag,
    external_tag,
    shape_index,
    rigidbody_tag,
    null_constraint,
    gravity_constraint,
    point_constraint,
    distance_constraint,
    soft_distance_constraint,
    hinge_constraint,
    generic_constraint,
    entity_owner
>{});

}

#endif // EDYN_NETWORKING_NETWORKED_COMP_HPP
