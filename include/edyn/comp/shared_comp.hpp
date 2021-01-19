#ifndef EDYN_SHARED_COMP_HPP
#define EDYN_SHARED_COMP_HPP

#include <tuple>
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
#include "edyn/comp/constraint.hpp"
#include "edyn/comp/constraint_row.hpp"
#include "edyn/comp/con_row_iter_data.hpp"
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
using shared_components = std::tuple<
    island, 
    island_node, 
    island_container,
    island_node_parent,
    island_node_child,
    island_timestamp,
    AABB, 
    angvel, 
    collision_filter, 
    constraint, 
    constraint_row, 
    con_row_iter_data,
    gravity, 
    inertia, 
    inertia_inv, 
    inertia_world_inv, 
    linacc,
    linvel, 
    mass, 
    mass_inv, 
    material, 
    orientation, 
    position,
    shape, 
    contact_manifold, 
    contact_point, 
    continuous,
    dynamic_tag, 
    kinematic_tag, 
    static_tag, 
    procedural_tag,
    sleeping_tag,
    sleeping_disabled_tag, 
    disabled_tag
>;

}

#endif // EDYN_SHARED_COMP_HPP