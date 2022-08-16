#ifndef EDYN_CONSTRAINTS_PREPARE_CONSTRAINTS_HPP
#define EDYN_CONSTRAINTS_PREPARE_CONSTRAINTS_HPP

#include <entt/entity/fwd.hpp>
#include "edyn/comp/orientation.hpp"
#include "edyn/math/matrix3x3.hpp"
#include "edyn/math/scalar.hpp"

namespace edyn {

struct constraint_row_prep_cache;
struct vector3;
struct quaternion;
struct matrix3x3;
struct position;
struct orientation;
struct origin;
class position_solver;

template<typename C>
void prepare_constraint(
    const entt::registry &registry, entt::entity entity, C &con,
    constraint_row_prep_cache &cache, scalar dt,
    const vector3 &originA, const vector3 &posA, const quaternion &ornA,
    const vector3 &linvelA, const vector3 &angvelA,
    scalar inv_mA, const matrix3x3 &inv_IA,
    const vector3 &originB, const vector3 &posB, const quaternion &ornB,
    const vector3 &linvelB, const vector3 &angvelB,
    scalar inv_mB, const matrix3x3 &inv_IB) {}

template<typename C>
void prepare_position_constraint(
    entt::registry &registry, entt::entity entity, C &con, position_solver &solver) {}

}

#endif // EDYN_CONSTRAINTS_PREPARE_CONSTRAINTS_HPP
