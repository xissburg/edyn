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
    const constraint_body &bodyA, const constraint_body &bodyB) {

    auto &settings = registry.ctx().get<edyn::settings>();

    // Create constraint rows for each contact point.
    for (unsigned pt_idx = 0; pt_idx < manifold.num_points; ++pt_idx) {
        auto &cp = manifold.get_point(pt_idx);

        EDYN_ASSERT(length_sqr(cp.normal) > EDYN_EPSILON);
        auto normal = cp.normal;
        auto pivotA = to_world_space(cp.pivotA, bodyA.origin, bodyA.orn);
        auto pivotB = to_world_space(cp.pivotB, bodyB.origin, bodyB.orn);
        auto rA = pivotA - bodyA.pos;
        auto rB = pivotB - bodyB.pos;

        // Create normal row, i.e. non-penetration constraint.
        auto &normal_row = cache.add_row();
        normal_row.J = {normal, cross(rA, normal), -normal, -cross(rB, normal)};
        normal_row.impulse = cp.normal_impulse;
        normal_row.lower_limit = 0;

        auto &normal_options = cache.get_options();

        // Do not use the traditional restitution path if the restitution solver
        // is being used.
        if (settings.num_restitution_iterations == 0) {
            normal_options.restitution = cp.restitution;
        }

        if (cp.distance < 0) {
            if (cp.stiffness < large_scalar) {
                auto vA = bodyA.linvel + cross(bodyA.angvel, rA);
                auto vB = bodyB.linvel + cross(bodyB.angvel, rB);
                auto relvel = vA - vB;
                auto normal_relvel = dot(relvel, normal);
                // Divide stiffness by number of points for correct force
                // distribution. All points have the same stiffness.
                auto spring_force = -cp.distance * cp.stiffness / manifold.num_points;
                auto damper_force = -normal_relvel * cp.damping / manifold.num_points;
                normal_row.upper_limit = std::max(spring_force + damper_force, scalar(0)) * dt;
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
            row_i.impulse = cp.friction_impulse[i];
            row_i.eff_mass = get_effective_mass(row_i.J, bodyA.inv_m, bodyA.inv_I, bodyB.inv_m, bodyB.inv_I);
            row_i.rhs = -get_relative_speed(row_i.J, bodyA.linvel, bodyA.angvel, bodyB.linvel, bodyB.angvel);
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
                        auto roll_dir = rotate(bodyA.orn, std::get<0>(roll_dir_view.get(body[j])));
                        axis *= dot(roll_dir, axis);
                    }
                }

                auto &row_i = roll_row.row[i];
                row_i.J = {vector3_zero, axis, vector3_zero, -axis};
                row_i.impulse = cp.rolling_friction_impulse[i];
                auto J_invM_JT = dot(bodyA.inv_I * row_i.J[1], row_i.J[1]) +
                                 dot(bodyB.inv_I * row_i.J[3], row_i.J[3]);
                row_i.eff_mass = J_invM_JT > EDYN_EPSILON ? scalar(1) / J_invM_JT : 0;
                row_i.rhs = -get_relative_speed(row_i.J, bodyA.linvel, bodyA.angvel, bodyB.linvel, bodyB.angvel);
            }
        }

        if (cp.spin_friction > 0) {
            auto &spin_row = cache.add_spinning_row();
            spin_row.friction_coefficient = cp.spin_friction;
            spin_row.J = {normal, -normal};
            spin_row.impulse = cp.spin_friction_impulse;

            auto J_invM_JT = dot(bodyA.inv_I * spin_row.J[0], spin_row.J[0]) +
                             dot(bodyB.inv_I * spin_row.J[1], spin_row.J[1]);
            EDYN_ASSERT(J_invM_JT > EDYN_EPSILON);
            spin_row.eff_mass = scalar(1) / J_invM_JT;
            spin_row.rhs = -(dot(spin_row.J[0], bodyA.angvel) + dot(spin_row.J[1], bodyB.angvel));
        }
    }
}

void contact_constraint::solve_position(position_solver &solver, contact_manifold &manifold) {
    // Solve position constraints by applying linear and angular corrections
    // iteratively. Based on Box2D's solver:
    // https://github.com/erincatto/box2d/blob/cd2c28dba83e4f359d08aeb7b70afd9e35e39eda/src/dynamics/b2_contact_solver.cpp#L676
    for (unsigned pt_idx = 0; pt_idx < manifold.num_points; ++pt_idx) {
        auto &cp = manifold.get_point(pt_idx);

        // Ignore soft contacts.
        if (cp.stiffness < large_scalar) {
            continue;
        }

        auto originA = solver.get_originA(), originB = solver.get_originB();
        auto &ornA = *solver.ornA, &ornB = *solver.ornB;
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

        auto &posA = *solver.posA, &posB = *solver.posB;
        auto rA = pivotA - posA;
        auto rB = pivotB - posB;

        if (cp.distance > -EDYN_EPSILON) {
            continue;
        }

        auto error = -cp.distance;
        solver.solve({normal, cross(rA, normal), -normal, -cross(rB, normal)}, error);
    }
}

void contact_constraint::store_applied_impulse(scalar impulse, unsigned row_index, contact_manifold &manifold) {
    EDYN_ASSERT(row_index < manifold.num_points);
    auto &cp = manifold.get_point(row_index);
    cp.normal_impulse = impulse;
}

void contact_constraint::store_friction_impulse(scalar impulse0, scalar impulse1, unsigned row_index, contact_manifold &manifold) {
    EDYN_ASSERT(row_index < manifold.num_points);
    auto &cp = manifold.get_point(row_index);
    cp.friction_impulse[0] = impulse0;
    cp.friction_impulse[1] = impulse1;
}

void contact_constraint::store_rolling_impulse(scalar impulse0, scalar impulse1, unsigned row_index, contact_manifold &manifold) {
    EDYN_ASSERT(row_index < manifold.num_points);
    auto &cp = manifold.get_point(row_index);
    cp.rolling_friction_impulse[0] = impulse0;
    cp.rolling_friction_impulse[1] = impulse1;
}

void contact_constraint::store_spinning_impulse(scalar impulse, unsigned row_index, contact_manifold &manifold) {
    EDYN_ASSERT(row_index < manifold.num_points);
    auto &cp = manifold.get_point(row_index);
    cp.spin_friction_impulse = impulse;
}

}
