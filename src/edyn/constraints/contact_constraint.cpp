#include "edyn/constraints/contact_constraint.hpp"
#include "edyn/comp/roll_direction.hpp"
#include "edyn/collision/contact_point.hpp"
#include "edyn/collision/contact_manifold.hpp"
#include "edyn/dynamics/position_solver.hpp"
#include "edyn/dynamics/row_cache.hpp"
#include "edyn/math/constants.hpp"
#include "edyn/math/transform.hpp"
#include "edyn/context/settings.hpp"
#include "edyn/util/collision_util.hpp"
#include <entt/entity/registry.hpp>

namespace edyn {

void contact_constraint::prepare(
    constraint_row_prep_cache &cache, scalar dt,
    const constraint_body &bodyA, const constraint_body &bodyB) {

    // Create constraint rows for each contact point.
    EDYN_ASSERT(length_sqr(normal) > EDYN_EPSILON);
    auto pivotA_world = to_world_space(pivotA, bodyA.origin, bodyA.orn);
    auto pivotB_world = to_world_space(pivotB, bodyB.origin, bodyB.orn);
    auto rA = pivotA_world - bodyA.pos;
    auto rB = pivotB_world - bodyB.pos;

    // Create normal row, i.e. non-penetration constraint.
    auto &normal_row = cache.add_row();
    normal_row.J = {normal, cross(rA, normal), -normal, -cross(rB, normal)};
    normal_row.impulse = applied_normal_impulse;
    normal_row.lower_limit = 0;
    normal_row.upper_limit = large_scalar;

    auto &normal_options = cache.get_options();
    normal_options.restitution = restitution;

    if (distance > 0) {
        // It is not penetrating thus apply an impulse that will prevent
        // penetration after the following physics update.
        normal_options.error = distance / dt;
    }

    // Create special friction rows.
    auto &friction_row = cache.add_friction_row();
    friction_row.friction_coefficient = friction;

    vector3 tangents[2];
    plane_space(normal, tangents[0], tangents[1]);

    for (auto i = 0; i < 2; ++i) {
        auto &row_i = friction_row.row[i];
        row_i.J = {tangents[i], cross(rA, tangents[i]), -tangents[i], -cross(rB, tangents[i])};
        row_i.impulse = applied_friction_impulse[i];
        row_i.eff_mass = get_effective_mass(row_i.J, bodyA.inv_m, bodyA.inv_I, bodyB.inv_m, bodyB.inv_I);
        row_i.rhs = -get_relative_speed(row_i.J, bodyA.linvel, bodyA.angvel, bodyB.linvel, bodyB.angvel);
    }
}

void contact_constraint::solve_position(position_solver &solver) {
    // Solve position constraints by applying linear and angular corrections
    // iteratively. Based on Box2D's solver:
    // https://github.com/erincatto/box2d/blob/cd2c28dba83e4f359d08aeb7b70afd9e35e39eda/src/dynamics/b2_contact_solver.cpp#L676
    auto originA = solver.get_originA(), originB = solver.get_originB();
    auto &ornA = *solver.ornA, &ornB = *solver.ornB;
    auto pivotA_world = to_world_space(pivotA, originA, ornA);
    auto pivotB_world = to_world_space(pivotB, originB, ornB);

    switch (normal_attachment) {
    case contact_normal_attachment::normal_on_A:
        normal = rotate(ornA, local_normal);
        break;
    case contact_normal_attachment::normal_on_B:
        normal = rotate(ornB, local_normal);
        break;
    case contact_normal_attachment::none:
        break;
    }

    distance = dot(pivotA_world - pivotB_world, normal);

    auto &posA = *solver.posA, &posB = *solver.posB;
    auto rA = pivotA_world - posA;
    auto rB = pivotB_world - posB;

    if (distance > -EDYN_EPSILON) {
        return;
    }

    auto error = -distance;
    solver.solve({normal, cross(rA, normal), -normal, -cross(rB, normal)}, error);
}

void contact_constraint::store_applied_impulses(const std::vector<scalar> &impulses) {
    unsigned row_idx = 0;

    applied_normal_impulse = impulses[row_idx++];
    applied_friction_impulse[0] = impulses[row_idx++];
    applied_friction_impulse[1] = impulses[row_idx++];
}

}
