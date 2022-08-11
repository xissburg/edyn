#include "edyn/constraints/contact_constraint.hpp"
#include "edyn/comp/tag.hpp"
#include "edyn/constraints/constraint_row.hpp"
#include "edyn/comp/position.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/comp/linvel.hpp"
#include "edyn/comp/angvel.hpp"
#include "edyn/comp/delta_linvel.hpp"
#include "edyn/comp/delta_angvel.hpp"
#include "edyn/comp/mass.hpp"
#include "edyn/comp/inertia.hpp"
#include "edyn/comp/origin.hpp"
#include "edyn/comp/roll_direction.hpp"
#include "edyn/collision/contact_point.hpp"
#include "edyn/collision/contact_manifold.hpp"
#include "edyn/dynamics/row_cache.hpp"
#include "edyn/math/constants.hpp"
#include "edyn/math/geom.hpp"
#include "edyn/math/math.hpp"
#include "edyn/math/transform.hpp"
#include "edyn/util/constraint_util.hpp"
#include "edyn/context/settings.hpp"
#include <entt/entity/registry.hpp>

namespace edyn {

template<>
void prepare_constraint<contact_constraint>(const entt::registry &registry, entt::entity entity, contact_constraint &con,
                                            constraint_row_prep_cache &cache, scalar dt,
                                            const vector3 &originA,
                                            const vector3 &posA, const quaternion &ornA,
                                            const vector3 &linvelA, const vector3 &angvelA,
                                            scalar inv_mA, const matrix3x3 &inv_IA,
                                            delta_linvel &dvA, delta_angvel &dwA,
                                            const vector3 &originB,
                                            const vector3 &posB, const quaternion &ornB,
                                            const vector3 &linvelB, const vector3 &angvelB,
                                            scalar inv_mB, const matrix3x3 &inv_IB,
                                            delta_linvel &dvB, delta_angvel &dwB) {
    auto &manifold = registry.get<contact_manifold>(entity);
    auto &settings = registry.ctx().at<edyn::settings>();

    // Create constraint rows for each contact point.
    for (unsigned pt_idx = 0; pt_idx < manifold.num_points; ++pt_idx) {
        auto &cp = manifold.get_point(pt_idx);

        EDYN_ASSERT(length_sqr(cp.normal) > EDYN_EPSILON);
        auto normal = cp.normal;
        auto pivotA = to_world_space(cp.pivotA, originA, ornA);
        auto pivotB = to_world_space(cp.pivotB, originB, ornB);
        auto rA = pivotA - posA;
        auto rB = pivotB - posB;

        // Create normal row, i.e. non-penetration constraint.
        auto &normal_row = cache.add_row();
        normal_row.J = {normal, cross(rA, normal), -normal, -cross(rB, normal)};
        normal_row.inv_mA = inv_mA; normal_row.inv_IA = inv_IA;
        normal_row.inv_mB = inv_mB; normal_row.inv_IB = inv_IB;
        normal_row.dvA = &dvA; normal_row.dwA = &dwA;
        normal_row.dvB = &dvB; normal_row.dwB = &dwB;
        normal_row.impulse = con.impulse[pt_idx];
        normal_row.lower_limit = 0;

        auto normal_options = constraint_row_options{};

        // Do not use the traditional restitution path if the restitution solver
        // is being used.
        if (settings.num_restitution_iterations == 0) {
            normal_options.restitution = cp.restitution;
        }

        if (cp.distance < 0) {
            if (cp.stiffness < large_scalar) {
                auto vA = linvelA + cross(angvelA, rA);
                auto vB = linvelB + cross(angvelB, rB);
                auto relvel = vA - vB;
                auto normal_relvel = dot(relvel, normal);
                // Divide stiffness by number of points for correct force
                // distribution. All points have the same stiffness.
                auto spring_force = cp.distance * cp.stiffness / manifold.num_points;
                auto damper_force = normal_relvel * cp.damping / manifold.num_points;
                normal_row.upper_limit = std::abs(spring_force + damper_force) * dt;
                normal_options.error = -large_scalar;
            } else {
                normal_row.upper_limit = large_scalar;
            }
        } else if (cp.stiffness >= large_scalar) {
            // It is not penetrating thus apply an impulse that will prevent
            // penetration after the following physics update.
            normal_options.error = cp.distance / dt;
            normal_row.upper_limit = large_scalar;
        }

        prepare_row(normal_row, normal_options, linvelA, angvelA, linvelB, angvelB);

        // Create special friction rows.
        auto &friction_row = cache.add_friction_row();
        friction_row.friction_coefficient = cp.friction;

        vector3 tangents[2];
        plane_space(normal, tangents[0], tangents[1]);

        for (auto i = 0; i < 2; ++i) {
            auto &row_i = friction_row.row[i];
            row_i.J = {tangents[i], cross(rA, tangents[i]), -tangents[i], -cross(rB, tangents[i])};
            row_i.impulse = con.impulse[max_contacts + pt_idx * 2 + i];
            row_i.eff_mass = get_effective_mass(row_i.J, inv_mA, inv_IA, inv_mB, inv_IB);
            row_i.rhs = -get_relative_speed(row_i.J, linvelA, angvelA, linvelB, angvelB);
        }

        if (cp.roll_friction > 0) {
            auto &roll_row = cache.add_rolling_row();
            roll_row.friction_coefficient = cp.roll_friction;

            auto roll_dir_view = registry.view<roll_direction>();

            for (auto i = 0; i < 2; ++i) {
                auto axis = tangents[i];

                // If any of the bodies has a rolling direction, scale down the
                // axis by the projection of the roll direction onto the axis,
                // thus preventing impulses in the undesired directions.
                for (auto j = 0; j < 2; ++j) {
                    if (roll_dir_view.contains(con.body[j])) {
                        auto roll_dir = rotate(ornA, std::get<0>(roll_dir_view.get(con.body[j])));
                        axis *= dot(roll_dir, axis);
                    }
                }

                auto &row_i = roll_row.row[i];
                row_i.J = {vector3_zero, axis, vector3_zero, -axis};
                row_i.impulse = con.impulse[max_contacts + max_contacts * 2 + pt_idx * 2 + i];
                auto J_invM_JT = dot(inv_IA * row_i.J[1], row_i.J[1]) +
                                 dot(inv_IB * row_i.J[3], row_i.J[3]);
                row_i.eff_mass = J_invM_JT > EDYN_EPSILON ? scalar(1) / J_invM_JT : 0;
                row_i.rhs = -get_relative_speed(row_i.J, linvelA, angvelA, linvelB, angvelB);
            }
        }

        if (cp.spin_friction > 0) {
            auto &spin_row = cache.add_spinning_row();
            spin_row.friction_coefficient = cp.spin_friction;
            spin_row.J = {normal, -normal};
            spin_row.impulse = cp.spin_friction_impulse;

            auto J_invM_JT = dot(inv_IA * spin_row.J[0], spin_row.J[0]) +
                             dot(inv_IB * spin_row.J[1], spin_row.J[1]);
            EDYN_ASSERT(J_invM_JT > EDYN_EPSILON);
            spin_row.eff_mass = scalar(1) / J_invM_JT;
            spin_row.rhs = -(dot(spin_row.J[0], angvelA) + dot(spin_row.J[1], angvelB));
        }
    }
}

template<>
bool solve_position_constraints<contact_constraint>(entt::registry &registry, scalar dt) {
    // Solve position constraints by applying linear and angular corrections
    // iteratively. Based on Box2D's solver:
    // https://github.com/erincatto/box2d/blob/cd2c28dba83e4f359d08aeb7b70afd9e35e39eda/src/dynamics/b2_contact_solver.cpp#L676

    // Remember that not all manifolds have a contact constraint.
    auto con_view = registry.view<contact_constraint, contact_manifold>(entt::exclude_t<sleeping_tag>{});
    auto body_view = registry.view<position, orientation, mass_inv, inertia_world_inv>();
    auto origin_view = registry.view<origin>();
    auto min_dist = scalar(0);

    for (auto entity : con_view) {
        auto &manifold = con_view.get<contact_manifold>(entity);

        if (manifold.num_points == 0) {
            continue;
        }

        // Ignore soft contacts.
        if (manifold.get_point(0).stiffness < large_scalar) {
            continue;
        }

        auto [posA, ornA, inv_mA, inv_IA] = body_view.get(manifold.body[0]);
        auto [posB, ornB, inv_mB, inv_IB] = body_view.get(manifold.body[1]);

        auto originA = origin_view.contains(manifold.body[0]) ? origin_view.get<origin>(manifold.body[0]) : static_cast<vector3>(posA);
        auto originB = origin_view.contains(manifold.body[1]) ? origin_view.get<origin>(manifold.body[1]) : static_cast<vector3>(posB);

        for (unsigned pt_idx = 0; pt_idx < manifold.num_points; ++pt_idx) {
            auto &cp = manifold.get_point(pt_idx);
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
        }

        auto basisA = to_matrix3x3(ornA);
        inv_IA = basisA * inv_IA * transpose(basisA);

        auto basisB = to_matrix3x3(ornB);
        inv_IB = basisB * inv_IB * transpose(basisB);
    }

    return min_dist > contact_position_solver_min_error;
}

}
