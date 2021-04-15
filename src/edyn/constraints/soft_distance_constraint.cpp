#include "edyn/constraints/soft_distance_constraint.hpp"
#include "edyn/comp/constraint.hpp"
#include "edyn/comp/constraint_row.hpp"
#include "edyn/comp/position.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/comp/linvel.hpp"
#include "edyn/comp/angvel.hpp"
#include "edyn/comp/delta_linvel.hpp"
#include "edyn/comp/delta_angvel.hpp"
#include "edyn/util/constraint_util.hpp"
#include "edyn/dynamics/row_cache.hpp"
#include <entt/entt.hpp>

namespace edyn {

void soft_distance_constraint::prepare(entt::entity entity, const constraint &con, 
                                       entt::registry &registry, row_cache &cache, 
                                       scalar dt) {
    auto &posA = registry.get<position>(con.body[0]);
    auto &ornA = registry.get<orientation>(con.body[0]);
    auto &posB = registry.get<position>(con.body[1]);
    auto &ornB = registry.get<orientation>(con.body[1]);

    auto rA = rotate(ornA, pivot[0]);
    auto rB = rotate(ornB, pivot[1]);

    auto d = posA + rA - posB - rB;
    auto l2 = length_sqr(d);
    auto l = std::sqrt(l2);
    vector3 dn;
    
    if (l2 > EDYN_EPSILON) {
        dn = d / l;
    } else {
        d = dn = vector3_x;
    }

    auto p = cross(rA, dn);
    auto q = cross(rB, dn);

    {
        // Spring row. By setting the error to +/- `large_scalar`, it will
        // always apply the impulse set in the limits.
        auto error = distance - l;
        auto spring_force = stiffness * error;
        auto spring_impulse = spring_force * dt;

        auto [row, data] = cache.make_row();
        data.J = {dn, p, -dn, -q};
        data.lower_limit = std::min(spring_impulse, scalar(0));
        data.upper_limit = std::max(scalar(0), spring_impulse);
        row.error = spring_impulse > 0 ? -large_scalar : large_scalar;
    }

    {
        // Damping row. It functions like friction where the force is
        // proportional to the relative speed.
        auto [row, data] = cache.make_row();
        data.J = {dn, p, -dn, -q};
        row.error = 0;

        auto &linvelA = registry.get<linvel>(con.body[0]);
        auto &angvelA = registry.get<angvel>(con.body[0]);
        auto &linvelB = registry.get<linvel>(con.body[1]);
        auto &angvelB = registry.get<angvel>(con.body[1]);

        auto relspd = dot(data.J[0], linvelA) + 
                      dot(data.J[1], angvelA) +
                      dot(data.J[2], linvelB) +
                      dot(data.J[3], angvelB);
        auto damping_force = damping * relspd;
        auto damping_impulse = damping_force * dt;
        auto impulse = std::abs(damping_impulse);

        data.lower_limit = -impulse;
        data.upper_limit =  impulse;

        m_relspd = relspd;

        m_damping_data = &data;
    }
}

void soft_distance_constraint::iteration(entt::entity entity, const constraint &con, 
                                         entt::registry &registry, row_cache &cache,
                                         size_t row_index, scalar dt) {
    // Adjust damping row limits to account for velocity changes during iterations.
    auto &dvA = registry.get<delta_linvel>(con.body[0]);
    auto &dwA = registry.get<delta_angvel>(con.body[0]);
    auto &dvB = registry.get<delta_linvel>(con.body[1]);
    auto &dwB = registry.get<delta_angvel>(con.body[1]);

    auto &data = cache.con_datas[row_index + 1];
    auto delta_relspd = dot(data.J[0], dvA) + 
                        dot(data.J[1], dwA) +
                        dot(data.J[2], dvB) +
                        dot(data.J[3], dwB);

    auto relspd = m_relspd + delta_relspd;

    auto damping_force = damping * relspd;
    auto damping_impulse = damping_force * dt;
    auto impulse = std::abs(damping_impulse);

    data.lower_limit = -impulse;
    data.upper_limit =  impulse;
}

}
