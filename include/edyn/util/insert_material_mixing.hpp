#ifndef EDYN_UTIL_INSERT_MATERIAL_MIXING_HPP
#define EDYN_UTIL_INSERT_MATERIAL_MIXING_HPP

#include <entt/entity/fwd.hpp>
#include "edyn/comp/material.hpp"

namespace edyn {

/**
 * @brief Use the provided material when two rigid bodies with the given
 * material ids collide.
 * @param registry Data source.
 * @param material_id0 ID of a material.
 * @param material_id1 ID of another material (could be equal to material_id0).
 * @param material Material info.
 */
void insert_material_mixing(entt::registry &registry, material::id_type material_id0,
                            material::id_type material_id1, const material_base &material);

}

#endif // EDYN_UTIL_INSERT_MATERIAL_MIXING_HPP
