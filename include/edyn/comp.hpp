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
#include "comp/relation.hpp"
#include "comp/constraint.hpp"
#include "comp/constraint_row.hpp"
#include "comp/gravity.hpp"
#include "comp/tag.hpp"
#include "comp/shape.hpp"
#include "comp/material.hpp"
#include "comp/island.hpp"
#include "comp/collision_filter.hpp"

namespace edyn {

using all_components = std::tuple<
    AABB, angvel, collision_filter, constraint, constraint_row, gravity, 
    inertia, inertia_inv, inertia_world_inv, island, island_node, linacc,
    linvel, mass, mass_inv, material, orientation, position, relation,
    shape, dynamic_tag, kinematic_tag, static_tag, sleeping_tag,
    sleeping_disabled_tag, disabled_tag
>;

using transient_components = std::tuple<
    AABB, angvel, constraint_row, inertia_world_inv, linvel, orientation,
    position
>;

}

#endif // EDYN_COMP_HPP