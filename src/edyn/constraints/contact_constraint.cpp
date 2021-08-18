#include "edyn/constraints/contact_constraint.hpp"
#include "edyn/collision/collision_result.hpp"
#include "edyn/collision/contact_manifold.hpp"
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
#include "edyn/comp/gravity.hpp"
#include "edyn/collision/contact_point.hpp"
#include "edyn/constraints/constraint_impulse.hpp"
#include "edyn/dynamics/row_cache.hpp"
#include "edyn/math/geom.hpp"
#include "edyn/math/math.hpp"
#include "edyn/config/constants.hpp"
#include "edyn/util/constraint_util.hpp"
#include "edyn/dynamics/solver.hpp"
#include <entt/entity/registry.hpp>

namespace edyn {

struct row_start_index_contact_constraint {
    size_t value;
};

void solve_friction_row_pair(internal::contact_friction_row_pair &friction_row_pair, constraint_row &normal_row) {
    vector2 delta_impulse;
    vector2 impulse;
    auto &friction_rows = friction_row_pair.row;

    for (auto i = 0; i < 2; ++i) {
        auto &friction_row = friction_rows[i];
        auto delta_relspd = get_relative_speed(friction_row.J,
                                               *normal_row.dvA, *normal_row.dwA,
                                               *normal_row.dvB, *normal_row.dwB);
        delta_impulse[i] = (friction_row.rhs - delta_relspd) * friction_row.eff_mass;
        impulse[i] = friction_row.impulse + delta_impulse[i];
    }

    auto impulse_len_sqr = length_sqr(impulse);
    auto max_impulse_len = friction_row_pair.friction_coefficient * normal_row.impulse;

    // Limit impulse by normal load.
    if (impulse_len_sqr > square(max_impulse_len) && impulse_len_sqr > EDYN_EPSILON) {
        auto impulse_len = std::sqrt(impulse_len_sqr);
        impulse = impulse / impulse_len * max_impulse_len;

        for (auto i = 0; i < 2; ++i) {
            delta_impulse[i] = impulse[i] - friction_rows[i].impulse;
        }
    }

    // Apply delta impulse.
    for (auto i = 0; i < 2; ++i) {
        auto &friction_row = friction_rows[i];
        friction_row.impulse = impulse[i];

        *normal_row.dvA += normal_row.inv_mA * friction_row.J[0] * delta_impulse[i];
        *normal_row.dwA += normal_row.inv_IA * friction_row.J[1] * delta_impulse[i];
        *normal_row.dvB += normal_row.inv_mB * friction_row.J[2] * delta_impulse[i];
        *normal_row.dwB += normal_row.inv_IB * friction_row.J[3] * delta_impulse[i];
    }
}

bool solve_restitution_iteration(entt::registry &registry, scalar dt) {
    auto body_view = registry.view<position, orientation, linvel, angvel, mass_inv, inertia_world_inv, delta_linvel, delta_angvel>();
    auto cp_view = registry.view<contact_point>();
    auto imp_view = registry.view<constraint_impulse>();
    auto com_view = registry.view<center_of_mass>();
    auto restitution_view = registry.view<contact_manifold_with_restitution>();
    auto manifold_view = registry.view<contact_manifold>();

    auto processed = std::vector<bool>{};
    processed.assign(restitution_view.size(), false);

    const unsigned num_individual_restitution_iterations = 2;

    // Solve each manifold in isolation, prioritizing the ones with higher
    // relative velocity, since they're more likely to propagate velocity
    // to their neighbors.
    size_t num_processed = 0;
    size_t num_solved = 0;

    while (num_processed < restitution_view.size()) {
        // Find manifold with highest penetration velocity.
        auto min_relvel = EDYN_SCALAR_MAX;
        auto manifold_entity = entt::entity{entt::null};
        size_t manifold_index = 0;
        size_t selected_manifold_index = SIZE_MAX;

        for (auto entity : restitution_view) {
            if (processed[manifold_index]) {
                ++manifold_index;
                continue;
            }

            auto &manifold = manifold_view.get(entity);
            auto num_points = manifold.num_points();

            if (num_points == 0) {
                processed[manifold_index++] = true;
                ++num_processed;
                continue;
            }

            auto [posA, ornA, linvelA, angvelA] =
                body_view.get<position, orientation, linvel, angvel>(manifold.body[0]);
            auto [posB, ornB, linvelB, angvelB] =
                body_view.get<position, orientation, linvel, angvel>(manifold.body[1]);

            auto originA = static_cast<vector3>(posA);
            auto originB = static_cast<vector3>(posB);

            if (com_view.contains(manifold.body[0])) {
                auto &com = com_view.get(manifold.body[0]);
                originA = to_world_space(-com, posA, ornA);
            }

            if (com_view.contains(manifold.body[1])) {
                auto &com = com_view.get(manifold.body[1]);
                originB = to_world_space(-com, posB, ornB);
            }

            auto local_min_relvel = EDYN_SCALAR_MAX;

            for (size_t pt_idx = 0; pt_idx < num_points; ++pt_idx) {
                auto &cp = cp_view.get(manifold.point[pt_idx]);
                auto normal = cp.normal;
                auto pivotA = to_world_space(cp.pivotA, originA, ornA);
                auto pivotB = to_world_space(cp.pivotB, originB, ornB);
                auto rA = pivotA - posA;
                auto rB = pivotB - posB;
                auto vA = linvelA + cross(angvelA, rA);
                auto vB = linvelB + cross(angvelB, rB);
                auto relvel = vA - vB;
                auto normal_relvel = dot(relvel, normal);
                local_min_relvel = std::min(normal_relvel, local_min_relvel);
            }

            if (local_min_relvel < min_relvel) {
                min_relvel = local_min_relvel;
                manifold_entity = entity;
                selected_manifold_index = manifold_index;
            }

            ++manifold_index;
        }

        if (manifold_entity == entt::null) {
            return true;
        }

        // In order to prevent bodies from bouncing forever, calculate a minimum
        // penetration velocity that must be met for the restitution impulse to
        // be applied.
        auto &manifold = manifold_view.get(manifold_entity);
        auto relvel_threshold = scalar(-0.2);

        // Make the threshold proportional to the velocity attained after gravity
        // is applied. This ensures bodies won't start bouncing while resting on
        // a surface due to gravity acceleration.
        if (auto *g = registry.try_get<gravity>(manifold.body[0])) {
            relvel_threshold = length(*g) * dt * scalar(-1.5);
        } else if (auto *g = registry.try_get<gravity>(manifold.body[1])) {
            relvel_threshold = length(*g) * dt * scalar(-1.5);
        }

        if (min_relvel > relvel_threshold) {
            processed[selected_manifold_index] = true;
            ++num_processed;
            ++num_solved;
            continue;
        }

        // Solve non-penetration constraints for this manifold in isolation.
        auto [posA, ornA, linvelA, angvelA, inv_mA, inv_IA, dvA, dwA] =
            body_view.get<position, orientation, linvel, angvel, mass_inv, inertia_world_inv, delta_linvel, delta_angvel>(manifold.body[0]);
        auto [posB, ornB, linvelB, angvelB, inv_mB, inv_IB, dvB, dwB] =
            body_view.get<position, orientation, linvel, angvel, mass_inv, inertia_world_inv, delta_linvel, delta_angvel>(manifold.body[1]);

        auto originA = static_cast<vector3>(posA);
        auto originB = static_cast<vector3>(posB);

        if (com_view.contains(manifold.body[0])) {
            auto &com = com_view.get(manifold.body[0]);
            originA = to_world_space(-com, posA, ornA);
        }

        if (com_view.contains(manifold.body[1])) {
            auto &com = com_view.get(manifold.body[1]);
            originB = to_world_space(-com, posB, ornB);
        }

        // Create constraint rows for non-penetration constraints for each
        // contact point.
        auto normal_rows = std::array<constraint_row, max_contacts>{};
        auto friction_row_pairs = std::array<internal::contact_friction_row_pair, max_contacts>{};
        auto num_points = manifold.num_points();

        for (size_t pt_idx = 0; pt_idx < num_points; ++pt_idx) {
            auto point_entity = manifold.point[pt_idx];
            auto &cp = cp_view.get(point_entity);

            auto normal = cp.normal;
            auto pivotA = to_world_space(cp.pivotA, originA, ornA);
            auto pivotB = to_world_space(cp.pivotB, originB, ornB);
            auto rA = pivotA - posA;
            auto rB = pivotB - posB;

            auto &normal_row = normal_rows[pt_idx];
            normal_row.J = {normal, cross(rA, normal), -normal, -cross(rB, normal)};
            normal_row.inv_mA = inv_mA; normal_row.inv_IA = inv_IA;
            normal_row.inv_mB = inv_mB; normal_row.inv_IB = inv_IB;
            normal_row.dvA = &dvA; normal_row.dwA = &dwA;
            normal_row.dvB = &dvB; normal_row.dwB = &dwB;
            normal_row.lower_limit = 0;
            normal_row.upper_limit = large_scalar;

            auto normal_options = constraint_row_options{};
            normal_options.restitution = cp.restitution;

            prepare_row(normal_row, normal_options, linvelA, angvelA, linvelB, angvelB);

            auto &friction_row_pair = friction_row_pairs[pt_idx];
            friction_row_pair.friction_coefficient = cp.friction;

            vector3 tangents[2];
            plane_space(normal, tangents[0], tangents[1]);

            for (auto i = 0; i < 2; ++i) {
                auto &friction_row = friction_row_pair.row[i];
                friction_row.J = {tangents[i], cross(rA, tangents[i]), -tangents[i], -cross(rB, tangents[i])};
                friction_row.eff_mass = get_effective_mass(friction_row.J, inv_mA, inv_IA, inv_mB, inv_IB);
                friction_row.rhs = -get_relative_speed(friction_row.J, linvelA, angvelA, linvelB, angvelB);
            }
        }

        // Solve rows.
        for (unsigned iter = 0; iter < num_individual_restitution_iterations; ++iter) {
            for (size_t pt_idx = 0; pt_idx < num_points; ++pt_idx) {
                auto &normal_row = normal_rows[pt_idx];
                auto delta_impulse = solve(normal_row);
                apply_impulse(delta_impulse, normal_row);

                auto &friction_row_pair = friction_row_pairs[pt_idx];
                solve_friction_row_pair(friction_row_pair, normal_row);
            }
        }

        // Persist applied impulses in a separate index because this cannot be
        // mixed with the regular constraint. It would apply these as the warm
        // starting impulse which will cause it to apply corrective impulses to
        // decelerate the rigid bodies which are separating.
        for (size_t pt_idx = 0; pt_idx < num_points; ++pt_idx) {
            auto &imp = imp_view.get(manifold.point[pt_idx]);
            auto &normal_row = normal_rows[pt_idx];
            imp.values[3] = normal_row.impulse;

            auto &friction_row_pair = friction_row_pairs[pt_idx];

            for (auto i = 0; i < 2; ++i) {
                imp.values[4 + i] = friction_row_pair.row[i].impulse;
            }
        }

        // Apply delta velocities.
        linvelA += dvA;
        angvelA += dwA;
        dvA = vector3_zero;
        dwA = vector3_zero;

        linvelB += dvB;
        angvelB += dwB;
        dvB = vector3_zero;
        dwB = vector3_zero;

        // Mark as processed.
        processed[selected_manifold_index] = true;
        ++num_processed;
    }

    if (num_solved == restitution_view.size()) {
        return true;
    }

    return false;
}

template<>
void prepare_constraints<contact_constraint>(entt::registry &registry, row_cache &cache, scalar dt) {
    const unsigned num_restitution_iterations = 1;

    for (unsigned i = 0; i < num_restitution_iterations; ++i) {
        if (solve_restitution_iteration(registry, dt)) {
            break;
        }
    }

    auto body_view = registry.view<position, orientation, linvel, angvel, mass_inv, inertia_world_inv, delta_linvel, delta_angvel>();
    auto con_view = registry.view<contact_constraint, contact_point>();
    auto imp_view = registry.view<constraint_impulse>();
    auto com_view = registry.view<center_of_mass>();

    size_t start_idx = cache.rows.size();
    registry.ctx_or_set<row_start_index_contact_constraint>().value = start_idx;

    cache.rows.reserve(cache.rows.size() + con_view.size());

    auto &ctx = registry.ctx<internal::contact_constraint_context>();
    ctx.friction_rows.clear();
    ctx.friction_rows.reserve(con_view.size());

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

        // Create normal row, i.e. non-penetration constraint.
        auto &normal_row = cache.rows.emplace_back();
        normal_row.J = {normal, cross(rA, normal), -normal, -cross(rB, normal)};
        normal_row.inv_mA = inv_mA; normal_row.inv_IA = inv_IA;
        normal_row.inv_mB = inv_mB; normal_row.inv_IB = inv_IB;
        normal_row.dvA = &dvA; normal_row.dwA = &dwA;
        normal_row.dvB = &dvB; normal_row.dwB = &dwB;
        normal_row.impulse = imp.values[0];
        normal_row.lower_limit = 0;

        auto normal_options = constraint_row_options{};

        // Do not use the traditional restitution path if the restitution solver
        // is being used.
        if (num_restitution_iterations == 0) {
            normal_options.restitution = cp.restitution;
        }

        if (cp.distance < 0) {
            if (con.stiffness < large_scalar) {
                auto vA = linvelA + cross(angvelA, rA);
                auto vB = linvelB + cross(angvelB, rB);
                auto relvel = vA - vB;
                auto normal_relvel = dot(relvel, normal);
                auto spring_force = cp.distance * con.stiffness;
                auto damper_force = normal_relvel * con.damping;
                normal_row.upper_limit = std::abs(spring_force + damper_force) * dt;
            } else {
                normal_row.upper_limit = large_scalar;
            }
        } else if (con.stiffness >= large_scalar) {
            // It is not penetrating thus apply an impulse that will prevent
            // penetration after the following physics update.
            normal_options.error = cp.distance / dt;
            normal_row.upper_limit = large_scalar;
        }

        prepare_row(normal_row, normal_options, linvelA, angvelA, linvelB, angvelB);
        warm_start(normal_row);

        // Create special friction rows.
        auto &friction_rows = ctx.friction_rows.emplace_back();
        friction_rows.friction_coefficient = cp.friction;

        vector3 tangents[2];
        plane_space(normal, tangents[0], tangents[1]);

        for (auto i = 0; i < 2; ++i) {
            auto &friction_row = friction_rows.row[i];
            friction_row.J = {tangents[i], cross(rA, tangents[i]), -tangents[i], -cross(rB, tangents[i])};
            friction_row.impulse = imp.values[1 + i];
            friction_row.eff_mass = get_effective_mass(friction_row.J, inv_mA, inv_IA, inv_mB, inv_IB);
            friction_row.rhs = -get_relative_speed(friction_row.J, linvelA, angvelA, linvelB, angvelB);

            // Warm-starting.
            dvA += inv_mA * friction_row.J[0] * friction_row.impulse;
            dwA += inv_IA * friction_row.J[1] * friction_row.impulse;
            dvB += inv_mB * friction_row.J[2] * friction_row.impulse;
            dwB += inv_IB * friction_row.J[3] * friction_row.impulse;
        }

        cache.con_num_rows.push_back(1);
    });
}

template<>
void iterate_constraints<contact_constraint>(entt::registry &registry, row_cache &cache, scalar dt) {
    auto start_row_idx = registry.ctx<row_start_index_contact_constraint>().value;
    auto &ctx = registry.ctx<internal::contact_constraint_context>();
    auto num_rows = ctx.friction_rows.size();

    // Solve friction rows locally using a non-standard method where the impulse
    // is limited by the length of a 2D vector to assure a friction circle.
    // These are the same fundamental operations found in `edyn::solver` adapted
    // to couple the two friction constraints together.
    // It is interesting to note that all friction constraints will be solved
    // before the non-penetration constraints, i.e. they're not interleaved.
    // Solving the non-penetration constraints last helps minimize penetration
    // errors because there won't be additional errors introduced by other
    // constraints.
    for (size_t row_idx = 0; row_idx < num_rows; ++row_idx) {
        auto &normal_row = cache.rows[start_row_idx + row_idx];
        auto &friction_row_pair = ctx.friction_rows[row_idx];
        solve_friction_row_pair(friction_row_pair, normal_row);
    }
}

template<>
bool solve_position_constraints<contact_constraint>(entt::registry &registry, scalar dt) {
    // Solve position constraints by applying linear and angular corrections
    // iteratively. Based on Box2D's solver:
    // https://github.com/erincatto/box2d/blob/cd2c28dba83e4f359d08aeb7b70afd9e35e39eda/src/dynamics/b2_contact_solver.cpp#L676
    auto cp_view = registry.view<contact_point>();
    auto body_view = registry.view<position, orientation, mass_inv, inertia_world_inv>();
    auto com_view = registry.view<center_of_mass>();
    auto min_dist = scalar(0);

    cp_view.each([&] (contact_point &cp) {
        auto [posA, ornA, inv_mA, inv_IA] =
            body_view.get<position, orientation, mass_inv, inertia_world_inv>(cp.body[0]);
        auto [posB, ornB, inv_mB, inv_IB] =
            body_view.get<position, orientation, mass_inv, inertia_world_inv>(cp.body[1]);

        auto originA = static_cast<vector3>(posA);
        auto originB = static_cast<vector3>(posB);

        if (com_view.contains(cp.body[0])) {
            auto &com = com_view.get(cp.body[0]);
            originA = to_world_space(-com, posA, ornA);
        }

        if (com_view.contains(cp.body[1])) {
            auto &com = com_view.get(cp.body[1]);
            originB = to_world_space(-com, posB, ornB);
        }

        auto pivotA = to_world_space(cp.pivotA, originA, ornA);
        auto pivotB = to_world_space(cp.pivotB, originB, ornB);

        switch (cp.normal_attachment) {
        case contact_normal_attachment::normal_on_A:
            cp.normal = rotate(ornA, cp.local_normal);
            break;
        case contact_normal_attachment::normal_on_B:
            cp.normal = rotate(ornB, cp.local_normal);
            break;
        case contact_normal_attachment::none:
            break;
        }

        auto normal = cp.normal;
        cp.distance = dot(pivotA - pivotB, normal);
        min_dist = std::min(cp.distance, min_dist);

        auto rA = pivotA - posA;
        auto rB = pivotB - posB;
        auto J = std::array<vector3, 4>{normal, cross(rA, normal), -normal, -cross(rB, normal)};
        auto eff_mass = get_effective_mass(J, inv_mA, inv_IA, inv_mB, inv_IB);
        auto error = std::min(cp.distance, scalar(0));
        auto correction = -error * contact_position_correction_rate * eff_mass;

        posA += inv_mA * J[0] * correction;
        posB += inv_mB * J[2] * correction;

        // Use quaternion derivative to apply angular correction which should
        // be good enough for small angles.
        auto angular_correctionA = inv_IA * J[1] * correction;
        ornA += quaternion_derivative(ornA, angular_correctionA);
        ornA = normalize(ornA);

        auto angular_correctionB = inv_IB * J[3] * correction;
        ornB += quaternion_derivative(ornB, angular_correctionB);
        ornB = normalize(ornB);

        auto basisA = to_matrix3x3(ornA);
        inv_IA = basisA * inv_IA * transpose(basisA);

        auto basisB = to_matrix3x3(ornB);
        inv_IB = basisB * inv_IB * transpose(basisB);
    });

    return min_dist > contact_position_solver_min_error;
}

}
