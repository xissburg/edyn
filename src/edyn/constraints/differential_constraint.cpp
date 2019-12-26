#include "edyn/constraints/differential_constraint.hpp"

#include "edyn/comp/constraint.hpp"
#include "edyn/comp/constraint_row.hpp"
#include "edyn/comp/relation.hpp"
#include "edyn/comp/position.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/math/matrix3x3.hpp"
#include "edyn/math/math.hpp"
#include <entt/entt.hpp>

namespace edyn {

void differential_constraint::init(entt::entity, constraint &con, const relation &rel, entt::registry &registry) {
    EDYN_ASSERT(rel.entity[2] != entt::null);

    con.num_rows = 1;
    con.row[0] = registry.create();
    auto &row = registry.assign<constraint_row>(con.row[0]);
    row.entity = rel.entity;

    for (size_t i = 0; i < 3; ++i) {
        row.use_spin[i] = true;
    }
}

void differential_constraint::prepare(entt::entity, constraint &con, const relation &rel, entt::registry &registry, scalar dt) {
    auto &ornA = registry.get<const orientation>(rel.entity[0]);
    auto &ornB = registry.get<const orientation>(rel.entity[1]);

    auto axis0 = rotate(ornA, -vector3_x);
    auto axis1 = rotate(ornB, -vector3_x);

    auto &row = registry.get<constraint_row>(con.row[0]);
    row.J = {vector3_zero, axis0, 
             vector3_zero, axis1,
             vector3_zero, {scalar(2) / ratio, 0, 0}};
    row.error = 0;
    row.lower_limit = -large_scalar;
    row.upper_limit = large_scalar;
}

}