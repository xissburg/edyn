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
void prepare_constraint<gravity_constraint>(
    const entt::registry &, entt::entity, gravity_constraint &con,
    constraint_row_prep_cache &cache, scalar dt,
    const vector3 &originA, const vector3 &posA, const quaternion &ornA,
    const vector3 &linvelA, const vector3 &angvelA,
    scalar inv_mA, const matrix3x3 &inv_IA,
    const vector3 &originB, const vector3 &posB, const quaternion &ornB,
    const vector3 &linvelB, const vector3 &angvelB,
    scalar inv_mB, const matrix3x3 &inv_IB) {

    auto d = posA - posB;
    auto l2 = length_sqr(d);
    l2 = std::max(l2, EDYN_EPSILON);

    auto l = std::sqrt(l2);
    auto dn = d / l;

    auto F = gravitational_constant / (l2 * inv_mA * inv_mB);
    auto P = F * dt;

    auto &row = cache.add_row();
    row.J = {dn, vector3_zero, -dn, -vector3_zero};
    row.lower_limit = -P;
    row.upper_limit = P;
    row.impulse = con.impulse;

    auto &options = cache.get_options();
    options.error = large_scalar;
}

}
