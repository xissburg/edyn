#include "edyn/constraints/distance_constraint.hpp"
#include "edyn/comp/constraint.hpp"
#include "edyn/comp/constraint_row.hpp"
#include "edyn/comp/position.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/util/constraint_util.hpp"
#include <entt/entt.hpp>

namespace edyn {

void distance_constraint::init(entt::entity entity, constraint &con, entt::registry &registry) {
    add_constraint_row(entity, con, registry, 400);
}

void distance_constraint::prepare(entt::entity, constraint &con, entt::registry &registry, scalar dt) {
    auto &posA = registry.get<position>(con.body[0]);
    auto &ornA = registry.get<orientation>(con.body[0]);
    auto &posB = registry.get<position>(con.body[1]);
    auto &ornB = registry.get<orientation>(con.body[1]);

    auto rA = rotate(ornA, pivot[0]);
    auto rB = rotate(ornB, pivot[1]);

    auto d = posA + rA - posB - rB;
    auto l2 = length_sqr(d);
    
    if (l2 <= EDYN_EPSILON) {
        d = vector3_x;
    }

    auto &data = registry.get<constraint_row_data>(con.row[0]);
    data.J = {d, cross(rA, d), -d, -cross(rB, d)};
    data.lower_limit = -large_scalar;
    data.upper_limit =  large_scalar;
    auto &row = registry.get<constraint_row>(con.row[0]);
    row.error = scalar(0.5) * (l2 - distance * distance) / dt;
}

}
