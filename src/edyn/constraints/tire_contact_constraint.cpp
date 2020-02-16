#include "edyn/constraints/tire_contact_constraint.hpp"
#include "edyn/comp/relation.hpp"
#include "edyn/comp/contact_manifold.hpp"
#include "edyn/comp/constraint.hpp"
#include "edyn/comp/constraint_row.hpp"
#include "edyn/comp/position.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/comp/shape.hpp"
#include "edyn/comp/linvel.hpp"
#include "edyn/comp/angvel.hpp"
#include "edyn/comp/spin.hpp"
#include "edyn/comp/material.hpp"
#include "edyn/math/constants.hpp"
#include "edyn/math/matrix3x3.hpp"
#include "edyn/util/array.hpp"

#include <entt/entt.hpp>

namespace edyn {

void tire_contact_constraint::prepare(entt::entity entity, constraint &con, const relation &rel, entt::registry &registry, scalar dt) {
    auto &posA = registry.get<const position   >(rel.entity[0]);
    auto &ornA = registry.get<const orientation>(rel.entity[0]);
    auto &posB = registry.get<const position   >(rel.entity[1]);
    auto &ornB = registry.get<const orientation>(rel.entity[1]);
    
    auto &linvelA = registry.get<const linvel>(rel.entity[0]);
    auto &angvelA = registry.get<const angvel>(rel.entity[0]);
    auto &linvelB = registry.get<const linvel>(rel.entity[1]);
    auto &angvelB = registry.get<const angvel>(rel.entity[1]);

    auto spinvelA = vector3_zero;
    auto spinvelB = vector3_zero;

    if (auto s = registry.try_get<const spin>(rel.entity[0])) {
        auto axis = rotate(ornA, vector3_x);
        spinvelA = axis * *s;
    }

    if (auto s = registry.try_get<const spin>(rel.entity[1])) {
        auto axis = rotate(ornB, vector3_x);
        spinvelB = axis * *s;
    }

    auto tire_x = rotate(ornA, vector3_x);
    auto tire_z = rotate(ornA, vector3_z);
    auto linvelrel = linvelA - linvelB;

    // Configure constraint rows.
    auto &manifold = registry.get<const contact_manifold>(entity);

    for (size_t i = 0; i < manifold.num_points; ++i) {
        auto &cp = manifold.point[i];

        auto rA = rotate(ornA, cp.pivotA);
        auto rB = rotate(ornB, cp.pivotB);
        auto normal = rotate(ornB, cp.normalB);

        auto vA = linvelA + cross(angvelA + spinvelA, rA);
        auto vB = linvelB + cross(angvelB + spinvelB, rB);
        auto relvel = vA - vB;
        auto normal_relvel = dot(relvel, normal);

        auto &normal_row = registry.get<constraint_row>(cp.normal_row_entity);
        normal_row.J = {normal, cross(rA, normal), -normal, -cross(rB, normal)};
        normal_row.lower_limit = 0;

        auto speed = length(linvelrel - normal * dot(linvelrel, normal));
        auto stiffness = velocity_dependent_vertical_stiffness(spec.vertical_stiffness, speed);
        auto spring_force = cp.distance * stiffness;
        auto damper_force = normal_relvel * spec.vertical_damping;
        normal_row.upper_limit = std::abs(spring_force + damper_force) * dt;

        auto rel = posA + rA - posB - rB;
        auto penetration = dot(rel, normal);
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
                constexpr scalar contact_erp = 0.2;
                normal_row.error = std::min(pvel, scalar(0)) * contact_erp;
            //}
        }

        // Setup friction constraints.
        auto &contact = contacts[i];
        if (contact.row_entity[0] == entt::null) {
            for (size_t j = 0; j < 3; ++j) {
                auto ent = registry.create();
                registry.assign<constraint_row>(ent);
                contact.row_entity[j] = ent;
            }
        }

        // Calculate longitudinal and lateral friction directions
        // project axis on contact plane.
        auto lat_dir = tire_x - normal * dot(tire_x, normal);
        auto lat_dir_len2 = length2(lat_dir);
        vector3 lon_dir;

        if (lat_dir_len2 > EDYN_EPSILON) {
            lat_dir /= std::sqrt(lat_dir_len2);
            lon_dir = cross(lat_dir, normal);
        } else {
            lon_dir = tire_z - normal * dot(tire_z, normal);
            lon_dir = normalize(lon_dir);
            lat_dir = cross(normal, lon_dir);
        }

        contact.lon_dir = lon_dir;
        contact.lat_dir = lat_dir;

        // The sign of the applied impulse gets inverted sometimes (e.g. when 
        // reversing) so copy the calculated impulse sign onto it.
        contact.last_lateral_impulse = std::copysign(contact.last_applied_impulse.y, contact.last_impulse.y);

        // For the impulse application point on the wheel we use the vector from
        // the center of the wheel to the contact point on B, which accounts for
        // tire deflection.
        auto pB = posB + rB;
        auto relA = pB - posA;
        auto relB = rB;

        // Longitudinal.
        {
            auto p = cross(relA, lon_dir);
            auto q = cross(relB, lon_dir);
            auto impulse = std::abs(contact.last_impulse.x);

            auto &row = registry.get<constraint_row>(contact.row_entity[0]);
            row.J = {lon_dir, p, -lon_dir, -q};
            row.error = 0;
            row.lower_limit = -impulse;
            row.upper_limit = impulse;
            row.use_spin[0] = true;
            row.use_spin[1] = true;
        }

        // Lateral.
        {
            auto p = cross(relA, lat_dir);
            auto q = cross(relB, lat_dir);
            auto impulse = std::abs(contact.last_impulse.y);

            auto &row = registry.get<constraint_row>(contact.row_entity[1]);
            row.J = {lat_dir, p, -lat_dir, -q};
            row.error = 0;
            row.lower_limit = -impulse;
            row.upper_limit = impulse;
            row.use_spin[0] = true;
            row.use_spin[1] = true;
        }

        // Aligning moment.
        {
            auto impulse = std::abs(contact.last_impulse.z);
            
            auto &row = registry.get<constraint_row>(contact.row_entity[2]);
            row.J = {vector3_zero, normal, vector3_zero, -normal};
            row.error = 0;
            row.lower_limit = -impulse;
            row.upper_limit = impulse;
            row.use_spin[0] = true;
            row.use_spin[1] = true;
        }
    }
}

vector3 tire_contact_constraint::calculate_tire_impulses(
        const vector3 &posA, const quaternion &ornA,
        const vector3 &linvelA, const vector3 &angvelA,
        const vector3 &spinvelA,
        const vector3 &posB, const quaternion &ornB,
        const vector3 &linvelB, const vector3 &angvelB,
        const vector3 &spinvelB,
        const contact_point &cp, 
        tire_contact_constraint::tire_contact &contact, 
        scalar normal_impulse, scalar dt) {
    auto normal = rotate(ornB, cp.normalB);
    auto fz = std::max(normal_impulse / dt, scalar(1));
    auto deflection = std::max(scalar(0.01), -cp.distance);

    auto rB = rotate(ornB, cp.pivotB);
    auto pB = posB + rB;
    auto r = pB - posA;

    auto velA = linvelA + cross(angvelA + spinvelA, r);
    auto velB = linvelB + cross(angvelB + spinvelB, rB);
    auto relvel = velA - velB;
    auto normal_relvel = dot(relvel, normal);
    auto tangent_relvel = relvel - normal * normal_relvel;
    auto tangent_relspd = length(tangent_relvel);
    auto tangent = tangent_relspd > EDYN_EPSILON ? tangent_relvel / tangent_relspd : vector3_x;

    auto linvelrel = linvelA - linvelB;
    auto vsx = dot(tangent_relvel, contact.lon_dir);
    auto vsy = dot(tangent_relvel, contact.lat_dir);
    auto vx = dot(linvelrel, contact.lon_dir);
    auto kappa = std::abs(vx) > 0.001 ? -vsx/vx : -vsx;
    auto tan_alpha = std::abs(vx) > 0.001 ? vsy/vx : 0;
    auto alpha = std::atan(tan_alpha);

    auto tire_x = rotate(ornA, vector3_x);
    auto sin_camber = dot(tire_x, normal);
    auto yaw_rate = dot(angvelA, normal);
    auto speed = length(linvelrel - normal * dot(linvelrel, normal));

    tire_state state;
    state.vertical_load = fz;
    state.vertical_deflection = deflection;
    state.speed = speed;
    state.friction_coefficient = cp.friction;
    state.sin_camber = sin_camber;
    state.slip_angle = alpha;
    state.slip_ratio = kappa;
    state.yaw_rate = yaw_rate;

    calculate_tire_forces(spec, state);
    auto impulses = vector3 {state.Fx, state.Fy, state.Mz} * dt;

    contact.last_impulse = impulses;
    contact.slide_factor = state.slide_factor;
    contact.lon_slip = kappa;
    contact.lat_slip = tan_alpha;
    contact.deflection = deflection;
    contact.contact_patch_length = state.contact_patch_length;
    contact.contact_patch_width = state.contact_patch_width;
    contact.slip_angle = alpha;

    return impulses;
}

void tire_contact_constraint::iteration(entt::entity entity, constraint &con, const relation &rel, entt::registry &registry, scalar dt) {
    auto &posA = registry.get<const position   >(rel.entity[0]);
    auto &ornA = registry.get<const orientation>(rel.entity[0]);
    auto &posB = registry.get<const position   >(rel.entity[1]);
    auto &ornB = registry.get<const orientation>(rel.entity[1]);
    
    auto &linvelA = registry.get<const linvel>(rel.entity[0]);
    auto &angvelA = registry.get<const angvel>(rel.entity[0]);
    auto &linvelB = registry.get<const linvel>(rel.entity[1]);
    auto &angvelB = registry.get<const angvel>(rel.entity[1]);

    auto spinvelA = vector3_zero;
    auto spinvelB = vector3_zero;

    if (auto s = registry.try_get<const spin>(rel.entity[0])) {
        auto axis = rotate(ornA, vector3_x);
        spinvelA = axis * *s;
    }

    if (auto s = registry.try_get<const spin>(rel.entity[1])) {
        auto axis = rotate(ornB, vector3_x);
        spinvelB = axis * *s;
    }
    
    auto &manifold = registry.get<const contact_manifold>(entity);

    for (size_t i = 0; i < manifold.num_points; ++i) {
        auto &cp = manifold.point.at(i);
        auto &normal_row = registry.get<constraint_row>(cp.normal_row_entity);
        auto &contact = contacts[i];
        auto impulses = calculate_tire_impulses(cp, contact, normal_row.impulse, dt);

        for (size_t j = 0; j < 3; ++j) {
            auto &row = registry.get<constraint_row>(contact.row_entity[j]);
            row.lower_limit = -impulses[j];
            row.upper_limit = impulses[j];
        }
    }
}

}