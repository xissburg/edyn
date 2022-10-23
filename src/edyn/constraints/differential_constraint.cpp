#include "edyn/constraints/differential_constraint.hpp"
#include "edyn/constraints/constraint_row.hpp"
#include "edyn/dynamics/row_cache.hpp"
#include "edyn/comp/position.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/comp/mass.hpp"
#include "edyn/comp/inertia.hpp"
#include "edyn/comp/linvel.hpp"
#include "edyn/comp/angvel.hpp"
#include "edyn/comp/delta_linvel.hpp"
#include "edyn/comp/delta_angvel.hpp"
#include "edyn/comp/spin.hpp"
#include "edyn/math/matrix3x3.hpp"
#include "edyn/math/math.hpp"
#include "edyn/util/constraint_util.hpp"
#include <entt/entity/registry.hpp>

namespace edyn {

void differential_constraint::prepare(
    const entt::registry &, entt::entity,
    constraint_row_prep_cache &cache, scalar dt,
    const constraint_body &bodyA, const constraint_body &bodyB, const constraint_body &bodyC) {

    auto axis0 = rotate(bodyA.orn, vector3_x);
    auto axis1 = rotate(bodyB.orn, vector3_x);

    auto &row = cache.add_row_triple();
    row.J = {vector3_zero, axis0,
             vector3_zero, axis1,
             vector3_zero, -vector3{scalar(2) / ratio, 0, 0}};
    row.lower_limit = -large_scalar;
    row.upper_limit = large_scalar;

    row.impulse = applied_impulse;
    row.use_spin[0] = true;
    row.use_spin[1] = true;
    row.use_spin[2] = true;
    row.spin_axis[0] = axis0;
    row.spin_axis[1] = axis1;
    row.spin_axis[2] = vector3_x;
}

void differential_constraint::store_applied_impulses(const std::vector<scalar> &impulses) {
    applied_impulse = impulses[0];
}

}
