#include "edyn/constraints/gravity_constraint.hpp"
#include "edyn/constraints/constraint_row.hpp"
#include "edyn/comp/position.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/comp/mass.hpp"
#include "edyn/comp/inertia.hpp"
#include "edyn/comp/linvel.hpp"
#include "edyn/comp/angvel.hpp"
#include "edyn/comp/delta_linvel.hpp"
#include "edyn/comp/delta_angvel.hpp"
#include "edyn/comp/tag.hpp"
#include "edyn/dynamics/row_cache.hpp"
#include "edyn/util/constraint_util.hpp"
#include <entt/entity/registry.hpp>

namespace edyn {

template<>
void prepare_constraint<gravity_constraint>(const entt::registry &, entt::entity, gravity_constraint &con,
                                            row_cache_sparse::entry &cache_entry, scalar dt,
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

    auto d = posA - posB;
    auto l2 = length_sqr(d);
    l2 = std::max(l2, EDYN_EPSILON);

    auto l = std::sqrt(l2);
    auto dn = d / l;

    auto F = gravitational_constant / (l2 * inv_mA * inv_mB);
    auto P = F * dt;

    auto &row = cache_entry.add_row();
    row.J = {dn, vector3_zero, -dn, -vector3_zero};
    row.lower_limit = -P;
    row.upper_limit = P;

    row.inv_mA = inv_mA; row.inv_IA = inv_IA;
    row.inv_mB = inv_mB; row.inv_IB = inv_IB;
    row.dvA = &dvA; row.dwA = &dwA;
    row.dvB = &dvB; row.dwB = &dwB;
    row.impulse = con.impulse;

    auto options = constraint_row_options{};
    options.error = large_scalar;

    prepare_row(row, options, linvelA, angvelA, linvelB, angvelB);
    warm_start(row);
}

}
