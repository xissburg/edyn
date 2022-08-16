#ifndef EDYN_CONSTRAINTS_PREPARE_CONSTRAINTS_HPP
#define EDYN_CONSTRAINTS_PREPARE_CONSTRAINTS_HPP

#include <entt/entity/fwd.hpp>
#include "edyn/comp/delta_angvel.hpp"
#include "edyn/comp/delta_linvel.hpp"
#include "edyn/dynamics/row_cache.hpp"
#include "edyn/math/scalar.hpp"

namespace edyn {

struct row_cache;

template<typename C>
void prepare_constraint(const entt::registry &, entt::entity, C &con,
                        constraint_row_prep_cache &cache, scalar dt,
                        const vector3 &originA, const vector3
                        &posA, const quaternion &ornA,
                        const vector3 &linvelA, const vector3 &angvelA,
                        scalar inv_mA, const matrix3x3 &inv_IA,
                        delta_linvel &dvA, delta_angvel &dwA,
                        const vector3 &originB,
                        const vector3 &posB, const quaternion &ornB,
                        const vector3 &linvelB, const vector3 &angvelB,
                        scalar inv_mB, const matrix3x3 &inv_IB,
                        delta_linvel &dvB, delta_angvel &dwB) {}

template<typename C>
void prepare_position_constraint(
    entt::registry &registry, entt::entity entity, C &con,
    constraint_row_positional_prep_cache &cache,
    const vector3 &originA, position &posA, orientation &ornA, scalar inv_mA, inertia_world_inv &inv_IA,
    const vector3 &originB, position &posB, orientation &ornB, scalar inv_mB, inertia_world_inv &inv_IB);

}

#endif // EDYN_CONSTRAINTS_PREPARE_CONSTRAINTS_HPP
