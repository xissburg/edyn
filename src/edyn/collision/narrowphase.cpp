#include "edyn/collision/narrowphase.hpp"
#include "edyn/comp/constraint.hpp"
#include "edyn/comp/relation.hpp"
#include "edyn/comp/material.hpp"
#include "edyn/comp/tire_material.hpp"
#include "edyn/comp/position.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/comp/shape.hpp"
#include "edyn/comp/contact_manifold.hpp"
#include "edyn/comp/constraint_row.hpp"
#include "edyn/comp/tag.hpp"
#include "edyn/collision/collide.hpp"
#include "edyn/math/geom.hpp"
#include <entt/entt.hpp>

namespace edyn {

void refresh_manifold(contact_manifold &manifold,
                      const vector3 &posA, const quaternion &ornA, 
                      const vector3 &posB, const quaternion &ornB) {
    for (size_t i = 0; i < manifold.num_points; ++i) {
        auto &pt = manifold.point[i];
        auto pivotA_world = posA + rotate(ornA, pt.pivotA);
        auto pivotB_world = posB + rotate(ornB, pt.pivotB);
        auto normal_world = rotate(ornB, pt.normalB);
        pt.distance = dot(normal_world, pivotA_world - pivotB_world);
    }
}

narrowphase::narrowphase(entt::registry &reg)
    : registry(&reg)
{

}

void narrowphase::on_construct_broadphase_relation(entt::entity entity, entt::registry &registry, relation &rel) {
    registry.assign<contact_manifold>(entity);

    auto m0 = registry.try_get<material>(rel.entity[0]);
    auto m1 = registry.try_get<material>(rel.entity[1]);

    if (m0 && m1) {
        auto stiffness = 1 / (1 / m0->stiffness + 1 / m1->stiffness);
        auto damping = 1 / (1 / m0->damping + 1 / m1->damping);

        auto tire0 = registry.try_get<tire_material>(rel.entity[0]);
        auto tire1 = registry.try_get<tire_material>(rel.entity[1]);

        if (tire0 || tire1) {
            auto contact = contact_patch_constraint();
            // Contact patch is always a soft contact since it needs deflection.
            EDYN_ASSERT(m0->stiffness < large_scalar || m1->stiffness < large_scalar);
            contact.m_normal_stiffness = stiffness;
            contact.m_normal_damping = damping;
            contact.m_friction_coefficient = m0->friction * m1->friction;

            auto tire = tire0 ? tire0 : tire1;

            contact.m_speed_sensitivity = tire->speed_sensitivity;
            contact.m_load_sensitivity = tire->load_sensitivity;
            contact.m_lat_tread_stiffness = tire->lat_tread_stiffness;
            contact.m_lon_tread_stiffness = tire->lon_tread_stiffness;
            
            registry.assign<constraint>(entity, contact);

            // Swap relation to ensure the cylinder is in the first entity.
            auto &shapeB = registry.get<shape>(rel.entity[1]);
            if (std::holds_alternative<cylinder_shape>(shapeB.var)) {
                std::swap(rel.entity[0], rel.entity[1]);
            }
        } else {
            auto contact = contact_constraint();

            if (m0->stiffness < large_scalar || m1->stiffness < large_scalar) {
                contact.stiffness = stiffness;
                contact.damping = damping;
            }

            registry.assign<constraint>(entity, contact);
        }
    }
}

void narrowphase::update() {
    auto view = registry->view<relation, contact_manifold>();

    view.each([&] (entt::entity ent, relation &rel, contact_manifold &manifold) {
        if (registry->has<sleeping_tag>(rel.entity[0]) && 
            registry->has<sleeping_tag>(rel.entity[1])) {
            return;
        }

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

        refresh_manifold(manifold, posA, ornA, posB, ornB);
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
            auto dA = length_sqr(rp.pivotA - cp.pivotA);
            auto dB = length_sqr(rp.pivotB - cp.pivotB);

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
            std::array<vector3, max_contacts> pivots;
            std::array<scalar, max_contacts> distances;
            for (size_t i = 0; i < manifold.num_points; ++i) {
                pivots[i] = manifold.point[i].pivotA;
                distances[i] = manifold.point[i].distance;
            }

            auto idx = insert_index(pivots, distances, manifold.num_points, rp.pivotA, rp.distance);

            if (idx < max_contacts) {
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
                        if (idx == manifold.num_points) {
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
                    }
                }

                if (idx == manifold.num_points) {
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
            length_sqr(dp) > contact_breaking_threshold * contact_breaking_threshold) {

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