#include "edyn/constraints/contact_constraint.hpp"
#include "edyn/comp/roll_direction.hpp"
#include "edyn/collision/contact_point.hpp"
#include "edyn/collision/contact_manifold.hpp"
#include "edyn/dynamics/position_solver.hpp"
#include "edyn/dynamics/row_cache.hpp"
#include "edyn/math/constants.hpp"
#include "edyn/math/transform.hpp"
#include "edyn/context/settings.hpp"
#include <entt/entity/registry.hpp>

namespace edyn {

void contact_constraint::prepare(
    const entt::registry &registry, entt::entity entity, const contact_manifold &manifold,
    constraint_row_prep_cache &cache, scalar dt,
    const vector3 &originA, const vector3 &posA, const quaternion &ornA,
    const vector3 &linvelA, const vector3 &angvelA,
    scalar inv_mA, const matrix3x3 &inv_IA,
    const vector3 &originB, const vector3 &posB, const quaternion &ornB,
    const vector3 &linvelB, const vector3 &angvelB,
    scalar inv_mB, const matrix3x3 &inv_IB) {

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
        normal_row.impulse = impulse[pt_idx];
        normal_row.lower_limit = 0;

        auto &normal_options = cache.get_options();

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

        // Create special friction rows.
        auto &friction_row = cache.add_friction_row();
        friction_row.friction_coefficient = cp.friction;

        vector3 tangents[2];
        plane_space(normal, tangents[0], tangents[1]);

        for (auto i = 0; i < 2; ++i) {
            auto &row_i = friction_row.row[i];
            row_i.J = {tangents[i], cross(rA, tangents[i]), -tangents[i], -cross(rB, tangents[i])};
            row_i.impulse = impulse[max_contacts + pt_idx * 2 + i];
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
                    if (roll_dir_view.contains(body[j])) {
                        auto roll_dir = rotate(ornA, std::get<0>(roll_dir_view.get(body[j])));
                        axis *= dot(roll_dir, axis);
                    }
                }

                auto &row_i = roll_row.row[i];
                row_i.J = {vector3_zero, axis, vector3_zero, -axis};
                row_i.impulse = impulse[max_contacts + max_contacts * 2 + pt_idx * 2 + i];
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

void contact_constraint::solve_position(position_solver &solver, contact_manifold &manifold) {
    // Solve position constraints by applying linear and angular corrections
    // iteratively. Based on Box2D's solver:
    // https://github.com/erincatto/box2d/blob/cd2c28dba83e4f359d08aeb7b70afd9e35e39eda/src/dynamics/b2_contact_solver.cpp#L676
    auto originA = solver.get_originA(), originB = solver.get_originB();
    auto &posA = *solver.posA, &posB = *solver.posB;
    auto &ornA = *solver.ornA, &ornB = *solver.ornB;

    for (unsigned pt_idx = 0; pt_idx < manifold.num_points; ++pt_idx) {
        auto &cp = manifold.get_point(pt_idx);

        // Ignore soft contacts.
        if (cp.stiffness < large_scalar) {
            continue;
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

        auto rA = pivotA - posA;
        auto rB = pivotB - posB;

        if (cp.distance > -EDYN_EPSILON) {
            continue;
        }

        auto error = -cp.distance;
        solver.solve({normal, cross(rA, normal), -normal, -cross(rB, normal)}, error);
    }
}

}
