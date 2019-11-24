#include "edyn/constraints/contact_constraint.hpp"

#include "edyn/comp/relation.hpp"
#include "edyn/comp/constraint.hpp"
#include "edyn/comp/constraint_row.hpp"
#include "edyn/comp/position.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/comp/shape.hpp"
#include "edyn/comp/linvel.hpp"
#include "edyn/comp/angvel.hpp"
#include "edyn/math/constants.hpp"
#include "edyn/math/matrix3x3.hpp"
#include "edyn/util/array.hpp"

#include <entt/entt.hpp>

namespace edyn {

void contact_constraint::init(constraint *con, const relation *rel, entt::registry &registry) {

}

void contact_constraint::prepare(constraint *con, const relation *rel, entt::registry &registry, scalar dt) {
    auto &posA   = registry.get<const position   >(rel->entity[0]);
    auto &ornA   = registry.get<const orientation>(rel->entity[0]);
    auto &shapeA = registry.get<const shape      >(rel->entity[0]);
    auto &posB   = registry.get<const position   >(rel->entity[1]);
    auto &ornB   = registry.get<const orientation>(rel->entity[1]);
    auto &shapeB = registry.get<const shape      >(rel->entity[1]);
    auto cm = contact_manifold {};

    // Perform narrow-phase collision detection.
    std::visit([&] (auto &&sA) {
        std::visit([&] (auto &&sB) {
            cm = collide(sA, posA, ornA, sB, posB, ornB);
        }, shapeB.var);
    }, shapeA.var);

    // Merge new with existing contact points.
    constexpr scalar contact_caching_threshold = 0.04;

    for (size_t i = 0; i < cm.num_points; ++i) {
        auto &cp1 = cm.point[i];
        scalar shortest_dist = contact_caching_threshold * contact_caching_threshold;
        size_t nearest_idx = max_contacts;

        for (size_t k = 0; k < manifold.num_points; ++k) {
            auto &cp0 = manifold.point[k];
            auto dA = length2(cp1.pivotA - cp0.pivotA);
            auto dB = length2(cp1.pivotB - cp0.pivotB);

            if (dA < shortest_dist) {
                shortest_dist = dA;
                nearest_idx = k;
            }

            if (dB < shortest_dist) {
                shortest_dist = dB;
                nearest_idx = k;
            }
        }

        if (nearest_idx < max_contacts) {
            auto &near_cp = manifold.point[nearest_idx];
            cp1.normal_row_entity = near_cp.normal_row_entity;
            cp1.friction_row_entity = near_cp.friction_row_entity;
            manifold.point[nearest_idx] = cp1;
        } else {
            // Create new constraint rows for this contact point.
            auto normal_row_entity = registry.create();
            con->row[con->num_rows++] = normal_row_entity;
            auto friction_row_entity = registry.create();
            con->row[con->num_rows++] = friction_row_entity;

            // Assign row component and associate entities.
            auto &normal_row = registry.assign<constraint_row>(normal_row_entity);
            normal_row.entity = rel->entity;
            auto &friction_row = registry.assign<constraint_row>(friction_row_entity);
            friction_row.entity = rel->entity;

            // Contact point can now refer to constraint rows.
            cp1.normal_row_entity = normal_row_entity;
            cp1.friction_row_entity = friction_row_entity;

            // Insert into array of points.
            auto insert_idx = manifold.num_points % max_contacts;
            manifold.point[insert_idx] = cp1;

            if (manifold.num_points < max_contacts) {
                ++manifold.num_points;
            }
        }
    }

    // Remove separating contact points.
    constexpr scalar contact_breaking_threshold = 0.04;

    for (size_t i = manifold.num_points; i > 0; --i) {
        size_t k = i - 1;
        auto &cp = manifold.point[k];
        auto pA = posA + rotate(ornA, cp.pivotA);
        auto pB = posB + rotate(ornB, cp.pivotB);
        auto n = rotate(ornB, cp.normalB);

        if (dot(pA - pB, n) > contact_breaking_threshold) {

            // Destroy constraint rows.
            for (int r = con->num_rows; r > 0; --r) {
                size_t s = r - 1;
                if (con->row[s] == cp.normal_row_entity || con->row[s] == cp.friction_row_entity) {
                    registry.destroy(con->row[s]);
                    // Swap with last element.
                    size_t t = con->num_rows - 1;
                    if (t != s) {
                        con->row[s] = con->row[t];
                    }
                    con->row[t] = entt::null;
                    --con->num_rows;
                }
            }

            // Swap with last element.
            size_t last_idx = manifold.num_points - 1;
            
            if (last_idx != k) {
                manifold.point[k] = manifold.point[last_idx];
            }

            --manifold.num_points;
        }
    }

    // Configure constraint rows.
    auto &linvelA = registry.get<const linvel>(rel->entity[0]);
    auto &angvelA = registry.get<const angvel>(rel->entity[0]);
    auto &linvelB = registry.get<const linvel>(rel->entity[1]);
    auto &angvelB = registry.get<const angvel>(rel->entity[1]);

    for (size_t i = 0; i < manifold.num_points; ++i) {
        auto &cp = manifold.point[i];

        auto rA = rotate(ornA, cp.pivotA);
        auto rB = rotate(ornB, cp.pivotB);
        auto normal = rotate(ornB, cp.normalB);

        auto vA = linvelA + cross(angvelA, rA);
        auto vB = linvelB + cross(angvelB, rB);
        auto relvel = vA - vB;
        auto normal_relvel = dot(relvel, normal);

        auto &normal_row = registry.get<constraint_row>(cp.normal_row_entity);
        normal_row.J = {normal, cross(rA, normal), -normal, -cross(rB, normal)};
        normal_row.error = -normal_relvel;
        normal_row.lower_limit = 0;
        normal_row.upper_limit = EDYN_SCALAR_MAX;

        /* auto penetration = -dot(posA + rA - posB - rB, normal);

        if (penetration > 0) {
            normal_row.error -= penetration / dt;
        } */

        auto tangent_relvel = relvel - normal * normal_relvel;
        auto tangent_relspd = length(tangent_relvel);
        auto tangent = tangent_relspd > EDYN_EPSILON ? tangent_relvel / tangent_relspd : vector3_x;

        auto &friction_row = registry.get<constraint_row>(cp.friction_row_entity);
        friction_row.J = {tangent, cross(rA, tangent), -tangent, -cross(rB, tangent)};
        friction_row.error = -tangent_relspd;
        // friction_row limits are calculated in `before_solve` using the normal impulse.
    }
}

void contact_constraint::before_solve(constraint *con, const relation *rel, entt::registry &registry, scalar dt) {
    auto &ornA = registry.get<const orientation>(rel->entity[0]);
    auto &ornB = registry.get<const orientation>(rel->entity[1]);

    auto &linvelA = registry.get<const linvel>(rel->entity[0]);
    auto &angvelA = registry.get<const angvel>(rel->entity[0]);
    auto &linvelB = registry.get<const linvel>(rel->entity[1]);
    auto &angvelB = registry.get<const angvel>(rel->entity[1]);

    for (size_t i = 0; i < manifold.num_points; ++i) {
        auto &cp = manifold.point[i];

        auto rA = rotate(ornA, cp.pivotA);
        auto rB = rotate(ornB, cp.pivotB);
        auto normal = rotate(ornB, cp.normalB);

        auto vA = linvelA + cross(angvelA, rA);
        auto vB = linvelB + cross(angvelB, rB);
        auto relvel = vA - vB;
        auto normal_relvel = dot(relvel, normal);

        constexpr scalar friction_coefficient = 0.8;

        auto &normal_row = registry.get<constraint_row>(cp.normal_row_entity);
        auto normal_impulse = normal_row.impulse;
        auto friction_impulse = normal_impulse * friction_coefficient;

        auto tangent_relvel = relvel - normal * normal_relvel;
        auto tangent_relspd = length(tangent_relvel);
        auto tangent = tangent_relspd > EDYN_EPSILON ? tangent_relvel / tangent_relspd : vector3_x;

        auto &friction_row = registry.get<constraint_row>(cp.friction_row_entity);
        friction_row.J = {tangent, cross(rA, tangent), -tangent, -cross(rB, tangent)};
        friction_row.error = -tangent_relspd;
        friction_row.lower_limit = -friction_impulse;
        friction_row.upper_limit = friction_impulse;
    }
}

}