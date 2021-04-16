#include "edyn/constraints/distance_constraint.hpp"
#include "edyn/comp/position.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/dynamics/row_cache.hpp"
#include <entt/entt.hpp>

namespace edyn {

void prepare_distance_constraints(entt::registry &registry, row_cache &cache, scalar dt) {
    auto body_view = registry.view<position, orientation>();
    auto con_view = registry.view<distance_constraint>();
    con_view.each([&] (distance_constraint &con) {
        auto [posA, ornA] = body_view.get<position, orientation>(con.body[0]);
        auto [posB, ornB] = body_view.get<position, orientation>(con.body[1]);

        auto rA = rotate(ornA, con.pivot[0]);
        auto rB = rotate(ornB, con.pivot[1]);

        auto d = posA + rA - posB - rB;
        auto l2 = length_sqr(d);
        
        if (l2 <= EDYN_EPSILON) {
            d = vector3_x;
        }

        auto [row, data] = cache.make_row();
        data.J = {d, cross(rA, d), -d, -cross(rB, d)};
        data.lower_limit = -large_scalar;
        data.upper_limit =  large_scalar;
        row.error = scalar(0.5) * (l2 - con.distance * con.distance) / dt;
    });
}

}
