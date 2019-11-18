#include <entt/entt.hpp>
#include "edyn/constraints/gravity_constraint.hpp"
#include "edyn/comp/position.hpp"
#include "edyn/comp/constraint.hpp"
#include "edyn/comp/constraint_row.hpp"
#include "edyn/comp/mass.hpp"
#include "edyn/math/constants.hpp"

namespace edyn {

void gravity_constraint::prepare(constraint *con, entt::registry &registry, scalar dt) {
    auto &posA = registry.get<const position>(con->entity[0]);
    auto &posB = registry.get<const position>(con->entity[1]);
    auto &row = registry.get<constraint_row>(con->row[0]);
    
    auto d = posA - posB;
    auto l2 = length2(d);
    l2 = std::max(l2, EDYN_EPSILON);

    auto l = std::sqrt(l2);
    auto dn = d / l;
    row.J = {-dn, vector3_zero, dn, vector3_zero};
    
    auto &mA = registry.get<const mass>(con->entity[0]);
    auto &mB = registry.get<const mass>(con->entity[1]);

    auto F = gravitational_constant * mA * mB / l2;
    auto P = std::min(F * dt, large_scalar);
    row.lower_limit = -P;
    row.upper_limit = P;
    row.error = large_scalar; // always apply maximum force 
}

}