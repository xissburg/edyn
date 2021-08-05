#include "edyn/constraints/contact_constraint.hpp"
#include "edyn/collision/collision_result.hpp"
#include "edyn/constraints/constraint_row.hpp"
#include "edyn/comp/position.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/comp/linvel.hpp"
#include "edyn/comp/angvel.hpp"
#include "edyn/comp/delta_linvel.hpp"
#include "edyn/comp/delta_angvel.hpp"
#include "edyn/comp/material.hpp"
#include "edyn/comp/mass.hpp"
#include "edyn/comp/inertia.hpp"
#include "edyn/comp/center_of_mass.hpp"
#include "edyn/collision/contact_point.hpp"
#include "edyn/constraints/constraint_impulse.hpp"
#include "edyn/dynamics/row_cache.hpp"
#include "edyn/math/quaternion.hpp"
#include "edyn/math/scalar.hpp"
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
        auto pivotA = to_world_space(cp.pivotA, originA, ornA);
        auto pivotB = to_world_space(cp.pivotB, originB, ornB);
        auto rA = pivotA - posA;
        auto rB = pivotB - posB;
        auto vA = linvelA + cross(angvelA, rA);
        auto vB = linvelB + cross(angvelB, rB);
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

        auto normal_options = constraint_row_options{};
        normal_options.restitution = cp.restitution;

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
        friction_row.dvA = &dvA; friction_row.dwA = &dwA;
        friction_row.dvB = &dvB; friction_row.dwB = &dwB;
        friction_row.impulse = imp.values[1];
        // friction_row limits are calculated in `iterate_contact_constraints`
        // using the normal impulse.
        friction_row.lower_limit = friction_row.upper_limit = 0;

        prepare_row(friction_row, {}, linvelA, linvelB, angvelA, angvelB);
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

template<>
bool solve_position_constraints<contact_constraint>(entt::registry &registry) {
    auto con_view = registry.view<contact_constraint, contact_point>();
    auto body_view = registry.view<position, orientation, linvel, angvel, mass_inv, inertia_world_inv, delta_linvel, delta_angvel>();
    auto com_view = registry.view<center_of_mass>();
    auto min_dist = scalar(0);

    con_view.each([&] (entt::entity entity, contact_constraint &con, contact_point &cp) {
        auto [posA, ornA, linvelA, angvelA, inv_mA, inv_IA, dvA, dwA] =
            body_view.get<position, orientation, linvel, angvel, mass_inv, inertia_world_inv, delta_linvel, delta_angvel>(con.body[0]);
        auto [posB, ornB, linvelB, angvelB, inv_mB, inv_IB, dvB, dwB] =
            body_view.get<position, orientation, linvel, angvel, mass_inv, inertia_world_inv, delta_linvel, delta_angvel>(con.body[1]);

        EDYN_ASSERT(con.body[0] == cp.body[0]);
        EDYN_ASSERT(con.body[1] == cp.body[1]);

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

        switch (cp.normal_attachment) {
        case contact_normal_attachment::normal_on_A:
            cp.normal = rotate(ornA, cp.local_normal);
            break;
        case contact_normal_attachment::normal_on_B:
            cp.normal = rotate(ornB, cp.local_normal);
            break;
        }

        auto normal = cp.normal;
        auto pivotA = to_world_space(cp.pivotA, originA, ornA);
        auto pivotB = to_world_space(cp.pivotB, originB, ornB);
        cp.distance = dot(pivotA - pivotB, normal);
        min_dist = std::min(cp.distance, min_dist);

        auto rA = pivotA - posA;
        auto rB = pivotB - posB;
        auto J = std::array<vector3, 4>{normal, cross(rA, normal), -normal, -cross(rB, normal)};
        auto J_invM_JT = dot(J[0], J[0]) * inv_mA +
                         dot(inv_IA * J[1], J[1]) +
                         dot(J[2], J[2]) * inv_mB +
                         dot(inv_IB * J[3], J[3]);
        auto eff_mass = scalar(1) / J_invM_JT;
        auto error = std::clamp((cp.distance + 0.005f) * 0.2f, -0.2f, 0.f);
        auto correction = error * eff_mass;

        posA += inv_mA * J[0] * correction;
        posB += inv_mB * J[2] * correction;

        auto angular_correctionA = inv_IA * J[1] * correction;
        auto angular_correctionA_len_sqr = length_sqr(angular_correctionA);

        if (angular_correctionA_len_sqr > EDYN_EPSILON) {
            auto angular_correctionA_len = std::sqrt(angular_correctionA_len_sqr);
            auto angular_correctionA_dir = angular_correctionA / angular_correctionA_len;
            ornA *= quaternion_axis_angle(angular_correctionA_dir, angular_correctionA_len);
        }

        auto angular_correctionB = inv_IB * J[3] * correction;
        auto angular_correctionB_len_sqr = length_sqr(angular_correctionB);

        if (angular_correctionB_len_sqr > EDYN_EPSILON) {
            auto angular_correctionB_len = std::sqrt(angular_correctionB_len_sqr);
            auto angular_correctionB_dir = angular_correctionB / angular_correctionB_len;
            ornB *= quaternion_axis_angle(angular_correctionB_dir, angular_correctionB_len);
        }
    });

    return min_dist > -0.015f;
}

}
