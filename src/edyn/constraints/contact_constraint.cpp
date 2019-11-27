#include "edyn/constraints/contact_constraint.hpp"

#include "edyn/comp/relation.hpp"
#include "edyn/comp/constraint.hpp"
#include "edyn/comp/constraint_row.hpp"
#include "edyn/comp/position.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/comp/shape.hpp"
#include "edyn/comp/linvel.hpp"
#include "edyn/comp/angvel.hpp"
#include "edyn/comp/matter.hpp"
#include "edyn/math/constants.hpp"
#include "edyn/math/matrix3x3.hpp"
#include "edyn/util/array.hpp"

#include <entt/entt.hpp>

namespace edyn {

scalar calc_restitution(const contact_point &cp, scalar dt) {
    // Higher restitution values decay faster.
    auto r1 = cp.restitution - 1;
    auto r = 2 - r1 * r1;
    // Higher delta times decay faster.
    auto s = dt * 120;
    // Exponential decay over lifetime of contact point.
    auto decay = std::exp(-scalar(cp.lifetime) * dt * r * s);
    return cp.restitution * decay;
}

void merge_point(const collision_result::collision_point &rp, contact_point &cp) {
    cp.pivotA = rp.pivotA;
    cp.pivotB = rp.pivotB;
    cp.normalB = rp.normalB;
}

void contact_constraint::process_collision(const collision_result &result, constraint &con, const relation &rel, entt::registry &registry) {
    // Merge new with existing contact points.
    for (size_t i = 0; i < result.num_points; ++i) {
        auto &rp = result.point[i];

        // Find closest existing point.
        scalar shortest_dist = contact_caching_threshold * contact_caching_threshold;
        size_t nearest_idx = max_contacts;

        for (size_t k = 0; k < manifold.num_points; ++k) {
            auto &cp = manifold.point[k];
            auto dA = length2(rp.pivotA - cp.pivotA);
            auto dB = length2(rp.pivotB - cp.pivotB);

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
            auto &cp = manifold.point[nearest_idx];
            ++cp.lifetime;
            merge_point(rp, cp);
        } else {
            // Create new constraint rows for this contact point.
            auto normal_row_entity = registry.create();
            con.row[con.num_rows++] = normal_row_entity;
            //auto friction_row_entity = registry.create();
            //con.row[con.num_rows++] = friction_row_entity;

            // Assign row component and associate entities.
            auto &normal_row = registry.assign<constraint_row>(normal_row_entity);
            normal_row.entity = rel.entity;
            //auto &friction_row = registry.assign<constraint_row>(friction_row_entity);
            //friction_row.entity = rel.entity;

            // Append to array of points and set it up.
            auto insert_idx = manifold.num_points % max_contacts;
            auto &cp = manifold.point[insert_idx];
            cp.lifetime = 0;
            merge_point(rp, cp);

            // Contact point can now refer to constraint rows.
            cp.normal_row_entity = normal_row_entity;
            //cp.friction_row_entity = friction_row_entity;

            // Combine matter/surface parameters.
            auto &matterA = registry.get<const matter>(rel.entity[0]);
            auto &matterB = registry.get<const matter>(rel.entity[1]);
            cp.restitution = matterA.restitution * matterB.restitution;
            cp.friction = matterA.friction * matterB.friction;

            if (manifold.num_points < max_contacts) {
                ++manifold.num_points;
            }
        }
    }
}

void contact_constraint::prune(const vector3 &posA, const quaternion &ornA, const vector3 &posB, const quaternion &ornB, constraint &con, entt::registry &registry) {
    // Remove separating contact points.
    for (size_t i = manifold.num_points; i > 0; --i) {
        size_t k = i - 1;
        auto &cp = manifold.point[k];
        auto pA = posA + rotate(ornA, cp.pivotA);
        auto pB = posB + rotate(ornB, cp.pivotB);
        auto n = rotate(ornB, cp.normalB);

        if (dot(pA - pB, n) > contact_breaking_threshold) {
            // Destroy constraint rows.
            for (int r = con.num_rows; r > 0; --r) {
                size_t s = r - 1;
                if (con.row[s] == cp.normal_row_entity) {// || con.row[s] == cp.friction_row_entity) {
                    registry.destroy(con.row[s]);
                    // Swap with last element.
                    size_t t = con.num_rows - 1;
                    if (t != s) {
                        con.row[s] = con.row[t];
                    }
                    --con.num_rows;
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
}

void contact_constraint::setup_rows(const vector3 &posA, const quaternion &ornA, const vector3 &posB, const quaternion &ornB, const relation &rel, entt::registry &registry, scalar dt) {
    // Configure constraint rows.
    auto &linvelA = registry.get<const linvel>(rel.entity[0]);
    auto &angvelA = registry.get<const angvel>(rel.entity[0]);
    auto &linvelB = registry.get<const linvel>(rel.entity[1]);
    auto &angvelB = registry.get<const angvel>(rel.entity[1]);

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
        normal_row.lower_limit = 0;
        normal_row.upper_limit = EDYN_SCALAR_MAX;

        auto restitution = calc_restitution(cp, dt);

        auto rel = posA + rA - posB - rB;
        auto penetration = dot(rel, normal);
        auto pvel = penetration / dt;

        // If not penetrating and the velocity necessary to touch in `dt` seconds
        // is smaller than the bounce velocity, it should apply an impulse that
        // will prevent penetration after the following physics update.
        if (penetration > 0 && pvel > -restitution * normal_relvel) {
            normal_row.error = std::max(pvel, scalar(0));
        } else {
            normal_row.error = restitution * normal_relvel;

            // If this is a resting contact and it is penetrating, apply impulse to push it out.
            if (cp.lifetime > 0) {
                constexpr scalar contact_erp = 0.2;
                normal_row.error += std::min(pvel, scalar(0)) * contact_erp;
            }
        }
        
        /* auto tangent_relvel = relvel - normal * normal_relvel;
        auto tangent_relspd = length(tangent_relvel);
        auto tangent = tangent_relspd > EDYN_EPSILON ? tangent_relvel / tangent_relspd : vector3_x;

        auto &friction_row = registry.get<constraint_row>(cp.friction_row_entity);
        friction_row.J = {tangent, cross(rA, tangent), -tangent, -cross(rB, tangent)};
        friction_row.error = 0;
        // friction_row limits are calculated in `before_solve` using the normal impulse.
        friction_row.lower_limit = -0;
        friction_row.upper_limit = 0; */
    }
}

void contact_constraint::prepare(constraint &con, const relation &rel, entt::registry &registry, scalar dt) {
    auto &posA   = registry.get<const position   >(rel.entity[0]);
    auto &ornA   = registry.get<const orientation>(rel.entity[0]);
    auto &shapeA = registry.get<const shape      >(rel.entity[0]);
    auto &posB   = registry.get<const position   >(rel.entity[1]);
    auto &ornB   = registry.get<const orientation>(rel.entity[1]);
    auto &shapeB = registry.get<const shape      >(rel.entity[1]);

    // Perform narrow-phase collision detection.
    auto result = collision_result {};
    std::visit([&] (auto &&sA) {
        std::visit([&] (auto &&sB) {
            result = collide(sA, posA, ornA, sB, posB, ornB, contact_breaking_threshold);
        }, shapeB.var);
    }, shapeA.var);

    process_collision(result, con, rel, registry);

    prune(posA, ornA, posB, ornB, con, registry);
    
    setup_rows(posA, ornA, posB, ornB, rel, registry, dt);
}

void contact_constraint::iteration(constraint &con, const relation &rel, entt::registry &registry, scalar dt) {
    /* for (size_t i = 0; i < manifold.num_points; ++i) {
        auto &cp = manifold.point[i];

        auto &normal_row = registry.get<constraint_row>(cp.normal_row_entity);
        auto friction_impulse = normal_row.impulse * cp.friction;

        auto &friction_row = registry.get<constraint_row>(cp.friction_row_entity);
        friction_row.lower_limit = -friction_impulse;
        friction_row.upper_limit = friction_impulse;
    } */
}

}