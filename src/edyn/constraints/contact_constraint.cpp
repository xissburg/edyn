#include "edyn/constraints/contact_constraint.hpp"
#include "edyn/comp/position.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/comp/linvel.hpp"
#include "edyn/comp/angvel.hpp"
#include "edyn/comp/delta_linvel.hpp"
#include "edyn/comp/delta_angvel.hpp"
#include "edyn/comp/material.hpp"
#include "edyn/comp/mass.hpp"
#include "edyn/comp/inertia.hpp"
#include "edyn/collision/contact_point.hpp"
#include "edyn/dynamics/row_cache.hpp"
#include "edyn/dynamics/solver.hpp"
#include <entt/entt.hpp>

namespace edyn {

struct row_start_index_contact_constraint {
    size_t value;
};

void prepare_contact_constraints(entt::registry &registry, row_cache &cache, scalar dt) {
    auto body_view = registry.view<position, orientation, linvel, angvel, mass_inv, inertia_world_inv, delta_linvel, delta_angvel>();
    auto con_view = registry.view<contact_constraint, contact_point>();
    size_t row_idx = cache.con_rows.size();
    registry.ctx_or_set<row_start_index_contact_constraint>(row_idx);

    con_view.each([&] (contact_constraint &con, contact_point &cp) {
        auto [posA, ornA, linvelA, angvelA] = body_view.get<position, orientation, linvel, angvel>(con.body[0]);
        auto [posB, ornB, linvelB, angvelB] = body_view.get<position, orientation, linvel, angvel>(con.body[1]);

        auto rA = rotate(ornA, cp.pivotA);
        auto rB = rotate(ornB, cp.pivotB);
        auto normal = rotate(ornB, cp.normalB);

        auto vA = linvelA + cross(angvelA, rA);
        auto vB = linvelB + cross(angvelB, rB);
        auto relvel = vA - vB;
        auto normal_relvel = dot(relvel, normal);

        auto [normal_row, normal_data] = cache.make_row();
        normal_data.J = {normal, cross(rA, normal), -normal, -cross(rB, normal)};
        normal_data.lower_limit = 0;
        normal_row.restitution = cp.restitution;

        if (con.stiffness < large_scalar) {
            auto spring_force = cp.distance * con.stiffness;
            auto damper_force = normal_relvel * con.damping;
            normal_data.upper_limit = std::abs(spring_force + damper_force) * dt;
        } else {
            normal_data.upper_limit = large_scalar;
        }

        auto penetration = dot(posA + rA - posB - rB, normal);
        auto pvel = penetration / dt;

        normal_row.error = 0;

        // If not penetrating and the velocity necessary to touch in `dt` seconds
        // is smaller than the bounce velocity, it should apply an impulse that
        // will prevent penetration after the following physics update.
        if (penetration > 0 && pvel > -cp.restitution * normal_relvel) {
            normal_row.error = std::max(pvel, scalar(0));
        } else {
            // If this is a resting contact and it is penetrating, apply impulse to push it out.
            //if (cp.lifetime > 0) {
                normal_row.error = std::min(pvel, scalar(0));
            //}
        }
        
        auto tangent_relvel = relvel - normal * normal_relvel;
        auto tangent_relspd = length(tangent_relvel);
        auto tangent = tangent_relspd > EDYN_EPSILON ? tangent_relvel / tangent_relspd : vector3_x;

        auto [friction_row, friction_data] = cache.make_row();
        friction_data.J = {tangent, cross(rA, tangent), -tangent, -cross(rB, tangent)};
        // friction_row limits are calculated in `iteration(...)` using the normal impulse.
        friction_data.lower_limit = friction_data.upper_limit = 0;
        friction_row.error = 0;

        con.m_friction = cp.friction;

        auto [inv_mA, inv_IA, dvA, dwA] = body_view.get<mass_inv, inertia_world_inv, delta_linvel, delta_angvel>(con.body[0]);
        auto [inv_mB, inv_IB, dvB, dwB] = body_view.get<mass_inv, inertia_world_inv, delta_linvel, delta_angvel>(con.body[1]);
        auto num_rows = 2;

        for (size_t i = 0; i < num_rows; ++i) {
            auto j = i + row_idx;
            const auto &row = cache.con_rows[j];
            auto &data = cache.con_datas[j];

            data.inv_mA = inv_mA;
            data.inv_mB = inv_mB;
            data.inv_IA = inv_IA;
            data.inv_IB = inv_IB;

            data.dvA = &dvA;
            data.dvB = &dvB;
            data.dwA = &dwA;
            data.dwB = &dwB;

            data.impulse = con.impulse[i];

            prepare_row(row, data, linvelA, linvelB, angvelA, angvelB);
            warm_start(data);
        }

        row_idx += num_rows;
        cache.con_num_rows.push_back(num_rows);
    });
}

void iterate_contact_constraints(entt::registry &registry, row_cache &cache, scalar dt) {
    auto con_view = registry.view<contact_constraint>();
    auto row_idx = registry.ctx<row_start_index_contact_constraint>().value;

    con_view.each([&] (contact_constraint &con) {
        const auto &normal_data = cache.con_datas[row_idx++];
        auto &friction_data = cache.con_datas[row_idx++];
        auto friction_impulse = std::abs(normal_data.impulse * con.m_friction);
        friction_data.lower_limit = -friction_impulse;
        friction_data.upper_limit = friction_impulse;
    });
}

}
