#include "edyn/constraints/contact_constraint.hpp"
#include "edyn/constraints/constraint_row.hpp"
#include "edyn/comp/position.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/comp/linvel.hpp"
#include "edyn/comp/angvel.hpp"
#include "edyn/comp/delta_linvel.hpp"
#include "edyn/comp/delta_angvel.hpp"
#include "edyn/comp/spin.hpp"
#include "edyn/comp/material.hpp"
#include "edyn/comp/mass.hpp"
#include "edyn/comp/inertia.hpp"
#include "edyn/comp/center_of_mass.hpp"
#include "edyn/collision/contact_point.hpp"
#include "edyn/constraints/constraint_impulse.hpp"
#include "edyn/dynamics/row_cache.hpp"
#include "edyn/util/constraint_util.hpp"
#include <entt/entity/registry.hpp>

namespace edyn {

struct row_start_index_contact_constraint {
    size_t value;
};

template<>
void prepare_constraints<contact_constraint>(entt::registry &registry, row_cache &cache, scalar dt) {
    auto body_view = registry.view<position, orientation, linvel, angvel, mass_inv, inertia_world_inv, delta_linvel, delta_angvel>();
    auto con_view = registry.view<contact_constraint, contact_point>();
    auto imp_view = registry.view<constraint_impulse>();
    auto com_view = registry.view<center_of_mass>();
    auto spin_view = registry.view<spin>();

    size_t start_idx = cache.rows.size();
    registry.ctx_or_set<row_start_index_contact_constraint>().value = start_idx;

    size_t num_rows_per_constraint = 2;
    cache.rows.reserve(cache.rows.size() + con_view.size() * num_rows_per_constraint);

    con_view.each([&] (entt::entity entity, contact_constraint &con, contact_point &cp) {
        auto [posA, ornA, linvelA, angvelA, inv_mA, inv_IA, dvA, dwA] =
            body_view.get<position, orientation, linvel, angvel, mass_inv, inertia_world_inv, delta_linvel, delta_angvel>(con.body[0]);
        auto [posB, ornB, linvelB, angvelB, inv_mB, inv_IB, dvB, dwB] =
            body_view.get<position, orientation, linvel, angvel, mass_inv, inertia_world_inv, delta_linvel, delta_angvel>(con.body[1]);
        auto &imp = imp_view.get(entity);

        EDYN_ASSERT(con.body[0] == cp.body[0]);
        EDYN_ASSERT(con.body[1] == cp.body[1]);

        auto spinvelA = vector3_zero;
        auto spinvelB = vector3_zero;

        if (spin_view.contains(con.body[0])) {
            auto &s = spin_view.get(con.body[0]);
            spinvelA = quaternion_x(ornA) * scalar(s);
        }

        if (spin_view.contains(con.body[1])) {
            auto &s = spin_view.get(con.body[1]);
            spinvelB = quaternion_x(ornB) * scalar(s);
        }

        auto originA = static_cast<vector3>(posA);
        auto originB = static_cast<vector3>(posB);

        if (com_view.contains(con.body[0])) {
            auto &com = com_view.get(con.body[0]);
            originA = to_world_space(-com, posA, ornA);
        }

        if (com_view.contains(con.body[1])) {
            auto &com = com_view.get(con.body[1]);
            originB = to_world_space(-com, posB, ornB);
        }

        auto normal = cp.normal;
        auto pA = to_world_space(cp.pivotA, originA, ornA);
        auto pB = to_world_space(cp.pivotB, originB, ornB);
        auto rA = pA - posA;
        auto rB = pB - posB;
        auto vA = linvelA + cross(angvelA + spinvelA, rA);
        auto vB = linvelB + cross(angvelB + spinvelB, rB);
        auto relvel = vA - vB;
        auto normal_relvel = dot(relvel, normal);

        // Create normal row.
        auto &normal_row = cache.rows.emplace_back();
        normal_row.J = {normal, cross(rA, normal), -normal, -cross(rB, normal)};
        normal_row.inv_mA = inv_mA; normal_row.inv_IA = inv_IA;
        normal_row.inv_mB = inv_mB; normal_row.inv_IB = inv_IB;
        normal_row.dvA = &dvA; normal_row.dwA = &dwA;
        normal_row.dvB = &dvB; normal_row.dwB = &dwB;
        normal_row.impulse = imp.values[0];
        normal_row.lower_limit = 0;

        if (con.stiffness < large_scalar) {
            auto spring_force = cp.distance * con.stiffness;
            auto damper_force = normal_relvel * con.damping;
            normal_row.upper_limit = std::abs(spring_force + damper_force) * dt;
        } else {
            normal_row.upper_limit = large_scalar;
        }

        auto penetration = dot(pA - pB, normal);
        auto pvel = penetration / dt;

        auto normal_options = constraint_row_options{};
        normal_options.restitution = cp.restitution;
        normal_options.error = 0;

        // If not penetrating and the velocity necessary to touch in `dt` seconds
        // is smaller than the bounce velocity, it should apply an impulse that
        // will prevent penetration after the following physics update.
        if (penetration > 0 && pvel > -cp.restitution * normal_relvel) {
            normal_options.error = std::max(pvel, scalar(0));
        } else {
            // If this is a resting contact and it is penetrating, apply impulse to push it out.
            //if (cp.lifetime > 0) {
                normal_options.error = std::min(pvel, scalar(0));
            //}
        }

        prepare_row(normal_row, normal_options, linvelA, linvelB, angvelA, angvelB);
        warm_start(normal_row);

        // Create friction row.
        auto tangent_relvel = relvel - normal * normal_relvel;
        auto tangent_relspd = length(tangent_relvel);
        auto tangent = tangent_relspd > EDYN_EPSILON ?
            tangent_relvel / tangent_relspd : vector3_x;

        auto &friction_row = cache.rows.emplace_back();
        friction_row.J = {tangent, cross(rA, tangent), -tangent, -cross(rB, tangent)};
        friction_row.inv_mA = inv_mA; friction_row.inv_IA = inv_IA;
        friction_row.inv_mB = inv_mB; friction_row.inv_IB = inv_IB;
        friction_row.dvA = &dvA; friction_row.dwA = &dwA; friction_row.dsA = registry.try_get<delta_spin>(con.body[0]);
        friction_row.dvB = &dvB; friction_row.dwB = &dwB; friction_row.dsB = registry.try_get<delta_spin>(con.body[1]);
        friction_row.impulse = imp.values[1];
        // friction_row limits are calculated in `iterate_contact_constraints`
        // using the normal impulse.
        friction_row.lower_limit = friction_row.upper_limit = 0;
        friction_row.use_spin[0] = true;
        friction_row.use_spin[1] = true;
        friction_row.spin_axis[0] = quaternion_x(ornA);
        friction_row.spin_axis[1] = quaternion_x(ornB);

        prepare_row(friction_row, {}, linvelA, linvelB, angvelA + spinvelA, angvelB + spinvelB);
        warm_start(friction_row);

        con.m_friction = cp.friction;

        cache.con_num_rows.push_back(num_rows_per_constraint);
    });
}

template<>
void iterate_constraints<contact_constraint>(entt::registry &registry, row_cache &cache, scalar dt) {
    auto con_view = registry.view<contact_constraint>();
    auto row_idx = registry.ctx<row_start_index_contact_constraint>().value;

    con_view.each([&] (contact_constraint &con) {
        const auto &normal_row = cache.rows[row_idx++];
        auto &friction_row = cache.rows[row_idx++];
        auto friction_impulse = std::abs(normal_row.impulse * con.m_friction);
        friction_row.lower_limit = -friction_impulse;
        friction_row.upper_limit = friction_impulse;
    });
}

}
