#ifndef EDYN_SHARED_COMP_HPP
#define EDYN_SHARED_COMP_HPP

#include "edyn/comp/aabb.hpp"
#include "edyn/comp/child_list.hpp"
#include "edyn/comp/gravity.hpp"
#include "edyn/comp/linvel.hpp"
#include "edyn/comp/angvel.hpp"
#include "edyn/comp/mass.hpp"
#include "edyn/comp/inertia.hpp"
#include "edyn/comp/position.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/comp/center_of_mass.hpp"
#include "edyn/comp/origin.hpp"
#include "edyn/constraints/constraint.hpp"
#include "edyn/comp/tag.hpp"
#include "edyn/comp/shape_index.hpp"
#include "edyn/comp/material.hpp"
#include "edyn/comp/island.hpp"
#include "edyn/comp/collision_filter.hpp"
#include "edyn/comp/collision_exclusion.hpp"
#include "edyn/comp/roll_direction.hpp"
#include "edyn/constraints/null_constraint.hpp"
#include "edyn/networking/comp/discontinuity.hpp"
#include "edyn/shapes/shapes.hpp"
#include "edyn/collision/contact_manifold.hpp"
#include "edyn/collision/contact_manifold_events.hpp"
#include "edyn/collision/contact_point.hpp"

namespace edyn {

/**
 * Tuple of components that are exchanged between main thread and
 * simulation worker.
 */
using shared_components_t = decltype(std::tuple_cat(std::tuple<
    AABB,
    island_AABB,
    collision_filter,
    collision_exclusion,
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
    contact_manifold,
    contact_manifold_with_restitution,
    contact_manifold_events,
    center_of_mass,
    origin,
    dynamic_tag,
    kinematic_tag,
    static_tag,
    procedural_tag,
    sleeping_tag,
    sleeping_disabled_tag,
    disabled_tag,
    external_tag,
    shape_index,
    island_resident,
    rigidbody_tag,
    constraint_tag,
    island_tag,
    rolling_tag,
    roll_direction,
    discontinuity_accumulator,
    child_list,
    parent_comp,
    null_constraint
>{}, constraints_tuple, shapes_tuple)); // Concatenate with all shapes and constraints at the end.

}

#endif // EDYN_SHARED_COMP_HPP
