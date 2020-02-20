#include "edyn/collision/narrowphase.hpp"
#include "edyn/comp/constraint.hpp"
#include "edyn/comp/relation.hpp"
#include "edyn/comp/material.hpp"
#include "edyn/comp/position.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/comp/shape.hpp"
#include "edyn/comp/contact_manifold.hpp"
#include "edyn/comp/constraint_row.hpp"
#include "edyn/collision/collide.hpp"
#include <entt/entt.hpp>

namespace edyn {

narrowphase::narrowphase(entt::registry &reg)
    : registry(&reg)
{

}

void narrowphase::on_construct_broadphase_relation(entt::entity entity, entt::registry &registry, relation &rel) {
    registry.assign<contact_manifold>(entity);

    auto m0 = registry.try_get<material>(rel.entity[0]);
    auto m1 = registry.try_get<material>(rel.entity[1]);

    if (m0 && m1) {
        if (m0->use_contact_patch || m1->use_contact_patch) {
            auto contact = contact_patch_constraint();
            // Contact patch is always a soft contact since it needs deflection.
            EDYN_ASSERT(m0->stiffness < large_scalar || m1->stiffness < large_scalar);
            contact.m_stiffness = 1 / (1 / m0->stiffness + 1 / m1->stiffness);
            contact.m_damping = 1 / (1 / m0->damping + 1 / m1->damping);
            contact.m_friction_coefficient = m0->friction * m1->friction;
            contact.m_speed_sensitivity = std::max(m0->speed_sensitivity, m1->speed_sensitivity);
            contact.m_tread_stiffness = std::max(m0->tread_stiffness, m1->tread_stiffness);
            registry.assign<constraint>(entity, contact);

            // Swap relation to ensure the cylinder is in the first entity.
            auto &shapeB = registry.get<shape>(rel.entity[1]);
            if (std::holds_alternative<cylinder_shape>(shapeB.var)) {
                std::swap(rel.entity[0], rel.entity[1]);
            }
        } else if (m0->is_tire || m1->is_tire) {
            auto contact = tire_contact_constraint();
            registry.assign<constraint>(entity, contact);
        } else {
            auto contact = contact_constraint();

            if (m0->stiffness < large_scalar || m1->stiffness < large_scalar) {
                contact.stiffness = 1 / (1 / m0->stiffness + 1 / m1->stiffness);
                contact.damping = 1 / (1 / m0->damping + 1 / m1->damping);
            }

            registry.assign<constraint>(entity, contact);
        }
    }
}

void narrowphase::update() {
    auto view = registry->view<relation, contact_manifold>();

    view.each([&] (entt::entity ent, relation &rel, contact_manifold &manifold) {
        auto &posA   = registry->get<const position   >(rel.entity[0]);
        auto &ornA   = registry->get<const orientation>(rel.entity[0]);
        auto &shapeA = registry->get<const shape      >(rel.entity[0]);
        auto &posB   = registry->get<const position   >(rel.entity[1]);
        auto &ornB   = registry->get<const orientation>(rel.entity[1]);
        auto &shapeB = registry->get<const shape      >(rel.entity[1]);

        auto result = collision_result{};
        std::visit([&] (auto &&sA) {
            std::visit([&] (auto &&sB) {
                result = collide(sA, posA, ornA, sB, posB, ornB, contact_breaking_threshold);
            }, shapeB.var);
        }, shapeA.var);

        process_collision(ent, manifold, rel, result);
        prune(ent, manifold, posA, ornA, posB, ornB);
    });
}

static
void merge_point(const collision_result::collision_point &rp, contact_point &cp) {
    cp.pivotA = rp.pivotA;
    cp.pivotB = rp.pivotB;
    cp.normalB = rp.normalB;
    cp.distance = rp.distance;
}

static
size_t insert_index(const contact_manifold &manifold, const collision_result::collision_point &rp) {
    if (manifold.num_points < max_contacts) {
        return manifold.num_points;
    }

    // Keep the deepest contact point.
    auto min_dist_idx = max_contacts;
    auto min_dist = rp.distance;

    for (size_t i = 0; i < max_contacts; ++i) {
        if (manifold.point[i].distance < min_dist) {
            min_dist = manifold.point[i].distance;
            min_dist_idx = i;
        }
    }

    // Find which combination maximizes the contact manifold area.
    auto areas = make_array<max_contacts>(scalar(0));
    size_t largest_idx = 0;
    scalar largest_area = 0;

    for (size_t i = 0; i < max_contacts; ++i) {
        if (i == min_dist_idx) {
            continue;
        }
        // Not exactly the area, could be incorrect in certain 
        // configurations but it's good enough.
        auto v = cross(rp.pivotA - manifold.point[(i + 1) % max_contacts].pivotA,
                       rp.pivotA - manifold.point[(i + 2) % max_contacts].pivotA);
        auto w = cross(manifold.point[(i + 3) % max_contacts].pivotA - manifold.point[(i + 1) % max_contacts].pivotA,
                        manifold.point[(i + 3) % max_contacts].pivotA - manifold.point[(i + 2) % max_contacts].pivotA);
        areas[i] = length2(v) + length2(w);
    
        if (areas[i] > largest_area) {
            largest_area = areas[i];
            largest_idx = i;
        }
    }

    return largest_idx;
}

void narrowphase::process_collision(entt::entity entity, contact_manifold &manifold, 
                                    const relation &rel, const collision_result &result) {
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
            // Append to array of points and set it up.
            auto idx = insert_index(manifold, rp);
            auto &cp = manifold.point[idx];
            cp.lifetime = 0;
            merge_point(rp, cp);

            if (auto con = registry->try_get<constraint>(entity)) {
                // Combine material/surface parameters.
                auto &materialA = registry->get<const material>(rel.entity[0]);
                auto &materialB = registry->get<const material>(rel.entity[1]);
                cp.restitution = materialA.restitution * materialB.restitution;
                cp.friction = materialA.friction * materialB.friction;

                // Only add normal and friction constraint rows if this is a
                // regular contact. Contact patches do their own thing.
                if (std::holds_alternative<contact_constraint>(con->var)) {
                    if (manifold.num_points < max_contacts) {
                        // Create new constraint rows for this contact point.
                        auto normal_row_entity = registry->create();
                        con->row[con->num_rows++] = normal_row_entity;
                        auto friction_row_entity = registry->create();
                        con->row[con->num_rows++] = friction_row_entity;

                        // Assign row component and associate entities.
                        auto &normal_row = registry->assign<constraint_row>(normal_row_entity);
                        normal_row.entity = rel.entity;
                        normal_row.priority = 0;
                        auto &friction_row = registry->assign<constraint_row>(friction_row_entity);
                        friction_row.entity = rel.entity;
                        friction_row.priority = 1;
                        friction_row.use_spin[0] = true;
                        friction_row.use_spin[1] = true;

                        // Contact point can now refer to constraint rows.
                        cp.normal_row_entity = normal_row_entity;
                        cp.friction_row_entity = friction_row_entity;

                        normal_row.restitution = cp.restitution;

                    } else {
                        // One of the existing contacts has been replaced by the new. 
                        // Update its rows.
                        auto &normal_row = registry->get<constraint_row>(cp.normal_row_entity);
                        auto &friction_row = registry->get<constraint_row>(cp.friction_row_entity);
                        normal_row.restitution = cp.restitution;

                        // Zero out warm-starting impulses.
                        normal_row.impulse = 0;
                        friction_row.impulse = 0;
                    }
                } else if (std::holds_alternative<tire_contact_constraint>(con->var)) {
                    if (manifold.num_points < max_contacts) {
                        // Create new constraint rows for this contact point.
                        auto normal_row_entity = registry->create();
                        con->row[con->num_rows++] = normal_row_entity;

                        // Assign row component and associate entities.
                        auto &normal_row = registry->assign<constraint_row>(normal_row_entity);
                        normal_row.entity = rel.entity;
                        normal_row.priority = 0;

                        // Contact point can now refer to constraint rows.
                        cp.normal_row_entity = normal_row_entity;
                        normal_row.restitution = cp.restitution;
                    } else {
                        // One of the existing contacts has been replaced by the new. 
                        // Update its rows.
                        auto &normal_row = registry->get<constraint_row>(cp.normal_row_entity);
                        normal_row.restitution = cp.restitution;

                        // Zero out warm-starting impulses.
                        normal_row.impulse = 0;
                    }
                }

                if (manifold.num_points < max_contacts) {
                    ++manifold.num_points;
                }
            }
        }
    }
}

void narrowphase::prune(entt::entity entity, contact_manifold &manifold,
                        const vector3 &posA, const quaternion &ornA, 
                        const vector3 &posB, const quaternion &ornB) {
    // Remove separating contact points.
    for (size_t i = manifold.num_points; i > 0; --i) {
        size_t k = i - 1;
        auto &cp = manifold.point[k];
        auto pA = posA + rotate(ornA, cp.pivotA);
        auto pB = posB + rotate(ornB, cp.pivotB);
        auto n = rotate(ornB, cp.normalB);
        auto d = pA - pB;
        auto dn = dot(d, n); // separation along normal
        auto dp = d - dn * n; // tangential separation on contact plane

        if (dn > contact_breaking_threshold ||
            length2(dp) > contact_breaking_threshold * contact_breaking_threshold) {

            if (auto con = registry->try_get<constraint>(entity)) {
                // Destroy constraint rows.
                for (int r = con->num_rows; r > 0; --r) {
                    size_t s = r - 1;
                    if (con->row[s] == cp.normal_row_entity || con->row[s] == cp.friction_row_entity) {
                        registry->destroy(con->row[s]);
                        // Swap with last element.
                        size_t t = con->num_rows - 1;
                        if (t != s) {
                            con->row[s] = con->row[t];
                        }
                        --con->num_rows;
                    }
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

}