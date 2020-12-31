#include "edyn/constraints/differential_constraint.hpp"

#include "edyn/comp/constraint.hpp"
#include "edyn/comp/constraint_row.hpp"
#include "edyn/comp/position.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/math/matrix3x3.hpp"
#include "edyn/math/math.hpp"
#include "edyn/util/constraint_util.hpp"
#include <entt/entt.hpp>

namespace edyn {

void differential_constraint::init(entt::entity entity, constraint &con, entt::registry &registry) {
    EDYN_ASSERT(con.body[2] != entt::null);

    auto row_entity = add_constraint_row(entity, con, registry, 100);
    auto &row = registry.get<constraint_row>(row_entity);

    for (size_t i = 0; i < 3; ++i) {
        row.use_spin[i] = true;
    }
}

void differential_constraint::prepare(entt::entity, constraint &con, 
                                      entt::registry &registry, scalar dt) {
    auto &ornA = registry.get<orientation>(con.body[0]);
    auto &ornB = registry.get<orientation>(con.body[1]);

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