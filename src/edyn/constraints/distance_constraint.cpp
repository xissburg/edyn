#include "edyn/constraints/distance_constraint.hpp"
#include "edyn/constraints/constraint_row.hpp"
#include "edyn/comp/position.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/comp/mass.hpp"
#include "edyn/comp/inertia.hpp"
#include "edyn/comp/linvel.hpp"
#include "edyn/comp/angvel.hpp"
#include "edyn/comp/delta_linvel.hpp"
#include "edyn/comp/delta_angvel.hpp"
#include "edyn/comp/origin.hpp"
#include "edyn/comp/tag.hpp"
#include "edyn/dynamics/row_cache.hpp"
#include "edyn/util/constraint_util.hpp"
#include "edyn/math/transform.hpp"
#include <entt/entity/registry.hpp>

namespace edyn {

template<>
void prepare_constraint<distance_constraint>(
    const entt::registry &, entt::entity, distance_constraint &con,
    constraint_row_prep_cache &cache, scalar dt,
    const vector3 &originA, const vector3 &posA, const quaternion &ornA,
    const vector3 &linvelA, const vector3 &angvelA,
    scalar inv_mA, const matrix3x3 &inv_IA,
    const vector3 &originB, const vector3 &posB, const quaternion &ornB,
    const vector3 &linvelB, const vector3 &angvelB,
    scalar inv_mB, const matrix3x3 &inv_IB) {

    auto pivotA = to_world_space(con.pivot[0], originA, ornA);
    auto pivotB = to_world_space(con.pivot[1], originB, ornB);
    auto rA = pivotA - posA;
    auto rB = pivotB - posB;

    auto d = pivotA - pivotB;
    auto dist_sqr = length_sqr(d);

    if (!(dist_sqr > EDYN_EPSILON)) {
        d = vector3_x;
    }

    auto &row = cache.add_row();
    row.J = {d, cross(rA, d), -d, -cross(rB, d)};
    row.lower_limit = -large_scalar;
    row.upper_limit =  large_scalar;
    row.impulse = con.impulse;

    auto &options = cache.get_options();
    options.error = scalar(0.5) * (dist_sqr - con.distance * con.distance) / dt;
}

}
