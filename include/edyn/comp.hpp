#ifndef EDYN_COMP_HPP
#define EDYN_COMP_HPP

#include <tuple>
#include "comp/aabb.hpp"
#include "comp/linacc.hpp"
#include "comp/linvel.hpp"
#include "comp/angvel.hpp"
#include "comp/mass.hpp"
#include "comp/inertia.hpp"
#include "comp/position.hpp"
#include "comp/orientation.hpp"
#include "comp/present_position.hpp"
#include "comp/present_orientation.hpp"
#include "comp/constraint.hpp"
#include "comp/constraint_row.hpp"
#include "comp/gravity.hpp"
#include "comp/tag.hpp"
#include "comp/shape.hpp"
#include "comp/material.hpp"
#include "comp/island.hpp"
#include "comp/collision_filter.hpp"
#include "edyn/collision/contact_manifold.hpp"
#include "edyn/collision/contact_point.hpp"

namespace edyn {

using all_components = std::tuple<
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
    dynamic_tag, 
    kinematic_tag, 
    static_tag, 
    procedural_tag,
    sleeping_tag,
    sleeping_disabled_tag, 
    disabled_tag
>;

using transient_components = std::tuple<
    AABB, angvel, constraint_row, inertia_world_inv, linvel, orientation,
    position, contact_point
>;

}

#endif // EDYN_COMP_HPP