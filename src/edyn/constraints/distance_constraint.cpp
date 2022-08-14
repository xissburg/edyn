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
void prepare_constraint<distance_constraint>(const entt::registry &, entt::entity, distance_constraint &con,
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
                                             delta_linvel &dvB, delta_angvel &dwB) {

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

    auto options = constraint_row_options{};
    options.error = scalar(0.5) * (dist_sqr - con.distance * con.distance) / dt;

    row.inv_mA = inv_mA; row.inv_IA = inv_IA;
    row.inv_mB = inv_mB; row.inv_IB = inv_IB;
    row.dvA = &dvA; row.dwA = &dwA;
    row.dvB = &dvB; row.dwB = &dwB;
    row.impulse = con.impulse;

    prepare_row(row, options, linvelA, angvelA, linvelB, angvelB);
}

}
