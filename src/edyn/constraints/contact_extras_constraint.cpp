#include "edyn/constraints/contact_extras_constraint.hpp"
#include "edyn/comp/roll_direction.hpp"
#include "edyn/config/config.h"
#include "edyn/dynamics/row_cache.hpp"
#include "edyn/math/constants.hpp"
#include "edyn/math/geom.hpp"
#include "edyn/math/transform.hpp"
#include "edyn/util/constraint_util.hpp"

namespace edyn {

void contact_extras_constraint::prepare(constraint_row_prep_cache &cache, scalar dt,
                                        const constraint_body &bodyA, const constraint_body &bodyB) {
    contact_constraint::prepare(cache, dt, bodyA, bodyB);

    if (distance < 0 && stiffness < large_scalar) {
        auto pivotA_world = to_world_space(pivotA, bodyA.origin, bodyA.orn);
        auto pivotB_world = to_world_space(pivotB, bodyB.origin, bodyB.orn);
        auto rA = pivotA_world - bodyA.pos;
        auto rB = pivotB_world - bodyB.pos;
        auto vA = bodyA.linvel + cross(bodyA.angvel, rA);
        auto vB = bodyB.linvel + cross(bodyB.angvel, rB);
        auto relvel = vA - vB;
        auto normal_relvel = dot(relvel, normal);
        // Divide stiffness by number of points for correct force
        // distribution. All points have the same stiffness.
        auto spring_force = -distance * stiffness / static_cast<scalar>(num_points);
        auto damper_force = -normal_relvel * damping / static_cast<scalar>(num_points);

        auto &normal_row = cache.get_current_row();
        normal_row.upper_limit = std::max(spring_force + damper_force, scalar(0)) * dt;

        auto &normal_options = cache.get_options();
        normal_options.error = -large_scalar;
    }

    if (roll_friction > 0) {
        auto &roll_row = cache.add_rolling_row();
        roll_row.friction_coefficient = roll_friction;

        vector3 tangents[2];
        plane_space(normal, tangents[0], tangents[1]);

        for (auto i = 0; i < 2; ++i) {
            auto axis = tangents[i];

            // If any of the bodies has a rolling direction, scale down the
            // axis by the projection of the roll direction onto the axis,
            // thus preventing impulses in the undesired directions.
            for (auto j = 0; j < 2; ++j) {
                if (roll_dir[j] != vector3_zero) {
                    auto roll_dir_world = rotate(bodyA.orn, roll_dir[j]);
                    axis *= dot(roll_dir_world, axis);
                }
            }

            auto &row_i = roll_row.row[i];
            row_i.J = {vector3_zero, axis, vector3_zero, -axis};
            row_i.impulse = rolling_friction_impulse[i];
            auto J_invM_JT = dot(bodyA.inv_I * row_i.J[1], row_i.J[1]) +
                                dot(bodyB.inv_I * row_i.J[3], row_i.J[3]);
            row_i.eff_mass = J_invM_JT > EDYN_EPSILON ? scalar(1) / J_invM_JT : 0;
            row_i.rhs = -get_relative_speed(row_i.J, bodyA.linvel, bodyA.angvel, bodyB.linvel, bodyB.angvel);
        }
    }

    if (spin_friction > 0) {
        auto &spin_row = cache.add_spinning_row();
        spin_row.friction_coefficient = spin_friction;
        spin_row.J = {normal, -normal};
        spin_row.impulse = spin_friction_impulse;

        auto J_invM_JT = dot(bodyA.inv_I * spin_row.J[0], spin_row.J[0]) +
                            dot(bodyB.inv_I * spin_row.J[1], spin_row.J[1]);
        EDYN_ASSERT(J_invM_JT > EDYN_EPSILON);
        spin_row.eff_mass = scalar(1) / J_invM_JT;
        spin_row.rhs = -(dot(spin_row.J[0], bodyA.angvel) + dot(spin_row.J[1], bodyB.angvel));
    }
}

void contact_extras_constraint::solve_position(position_solver &solver) {
    // Do not use position solver for soft contacts.
    if (stiffness >= large_scalar) {
        contact_constraint::solve_position(solver);
    }
}

void contact_extras_constraint::store_applied_impulses(const std::vector<scalar> &impulses) {
    contact_constraint::store_applied_impulses(impulses);

    unsigned row_idx = 3;

    if (roll_friction > 0) {
        rolling_friction_impulse[0] = impulses[row_idx++];
        rolling_friction_impulse[1] = impulses[row_idx++];
    }

    if (spin_friction > 0) {
        spin_friction_impulse = impulses[row_idx++];
    }

    EDYN_ASSERT(row_idx <= impulses.size());
}

}
