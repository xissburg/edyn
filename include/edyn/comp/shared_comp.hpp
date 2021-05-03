#ifndef EDYN_SHARED_COMP_HPP
#define EDYN_SHARED_COMP_HPP

#include "edyn/util/tuple_util.hpp"
#include "edyn/comp/aabb.hpp"
#include "edyn/comp/linacc.hpp"
#include "edyn/comp/linvel.hpp"
#include "edyn/comp/angvel.hpp"
#include "edyn/comp/mass.hpp"
#include "edyn/comp/inertia.hpp"
#include "edyn/comp/position.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/comp/present_position.hpp"
#include "edyn/comp/present_orientation.hpp"
#include "edyn/constraints/constraint.hpp"
#include "edyn/constraints/constraint_impulse.hpp"
#include "edyn/comp/gravity.hpp"
#include "edyn/comp/tag.hpp"
#include "edyn/comp/shape.hpp"
#include "edyn/comp/material.hpp"
#include "edyn/comp/island.hpp"
#include "edyn/comp/collision_filter.hpp"
#include "edyn/comp/continuous.hpp"
#include "edyn/collision/contact_manifold.hpp"
#include "edyn/collision/contact_point.hpp"

namespace edyn {

/**
 * Tuple of components that are exchanged between island coordinator and
 * island workers.
 */
using shared_components = tuple_cat_type<std::tuple<
    island_timestamp,
    AABB, 
    collision_filter, 
    constraint_impulse,
    point_constraint, 
    distance_constraint,
    soft_distance_constraint,
    hinge_constraint,
    generic_constraint,
    contact_constraint,
    gravity, 
    inertia, 
    inertia_inv, 
    inertia_world_inv, 
    linacc,
    angvel, 
    linvel, 
    mass, 
    mass_inv, 
    material, 
    position,
    orientation, 
    contact_manifold, 
    contact_point, 
    continuous,
    dynamic_tag, 
    kinematic_tag, 
    static_tag, 
    procedural_tag,
    sleeping_tag,
    sleeping_disabled_tag, 
    disabled_tag,
    continuous_contacts_tag,
    shape_index
>, shapes_tuple_t>::type;

}

#endif // EDYN_SHARED_COMP_HPP
