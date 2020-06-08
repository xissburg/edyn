#include "edyn/collision/narrowphase.hpp"
#include "edyn/comp/constraint.hpp"
#include "edyn/comp/relation.hpp"
#include "edyn/comp/material.hpp"
#include "edyn/comp/tire_material.hpp"
#include "edyn/comp/position.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/comp/shape.hpp"
#include "edyn/comp/constraint_row.hpp"
#include "edyn/comp/tag.hpp"
#include "edyn/collision/contact_manifold.hpp"
#include "edyn/collision/contact_point.hpp"
#include "edyn/collision/collide.hpp"
#include "edyn/math/geom.hpp"
#include "edyn/util/constraint.hpp"
#include <entt/entt.hpp>
#include <utility>

namespace edyn {

static
void refresh_contact_points(entt::registry &registry) {
    auto tr_view = registry.view<position, orientation>();
    auto cp_view = registry.view<contact_point>();
    auto rel_view = registry.view<relation>();
    cp_view.each([&] (auto, contact_point &cp) {
        auto &rel = rel_view.get(cp.parent);
        auto [posA, ornA] = tr_view.get<position, orientation>(rel.entity[0]);
        auto [posB, ornB] = tr_view.get<position, orientation>(rel.entity[1]);
        auto pivotA_world = posA + rotate(ornA, cp.pivotA);
        auto pivotB_world = posB + rotate(ornB, cp.pivotB);
        auto normal_world = rotate(ornB, cp.normalB);
        cp.distance = dot(normal_world, pivotA_world - pivotB_world);
    });
}

static
void on_destroy_manifold(entt::entity entity, entt::registry &registry) {
    auto &manifold = registry.get<contact_manifold>(entity);

    for (size_t i = 0; i < manifold.num_points; ++i) {
        registry.destroy(manifold.point_entity[i]);
    }
}

narrowphase::narrowphase(entt::registry &reg)
    : registry(&reg)
{
    connections.push_back(registry->on_destroy<contact_manifold>().connect<&on_destroy_manifold>());
}

void narrowphase::update() {
    refresh_contact_points(*registry);

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
                result = collide(sA, posA, ornA, sB, posB, ornB, 
                                 contact_breaking_threshold);
            }, shapeB.var);
        }, shapeA.var);

        process_collision(ent, manifold, rel, result);
        prune(ent, manifold, rel, posA, ornA, posB, ornB);
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
void create_contact_constraint(entt::entity manifold_entity, entt::entity contact_entity,
                               entt::registry &registry, contact_point &cp, relation &rel) {
    auto *materialA = registry.try_get<const material>(rel.entity[0]);
    auto *materialB = registry.try_get<const material>(rel.entity[1]);
    
    if (!materialA || !materialB) {
        return;
    }

    cp.restitution = materialA->restitution * materialB->restitution;
    cp.friction = materialA->friction * materialB->friction;

    auto stiffness = large_scalar;
    auto damping = large_scalar;

    if (materialA->stiffness < large_scalar || materialB->stiffness < large_scalar) {
        stiffness = 1 / (1 / materialA->stiffness + 1 / materialB->stiffness);
        damping = 1 / (1 / materialA->damping + 1 / materialB->damping);
    }

    auto tire0 = registry.try_get<tire_material>(rel.entity[0]);
    auto tire1 = registry.try_get<tire_material>(rel.entity[1]);
    auto is_tire = tire0 || tire1;

    if (is_tire) {
        // For tires a `contact_patch_constraint` is assigned to the manifold
        // entity. It creates constraints internally if the number of points
        // is greater than zero.
        if (!registry.has<constraint>(manifold_entity)) {
            auto contact = contact_patch_constraint();
            // Contact patch is always a soft contact since it needs deflection
            // to generate friction forces.
            EDYN_ASSERT(stiffness < large_scalar || stiffness < large_scalar);
            contact.m_normal_stiffness = stiffness;
            contact.m_normal_damping = damping;
            contact.m_friction_coefficient = materialA->friction * materialB->friction;

            auto tire = tire0 ? tire0 : tire1;

            contact.m_speed_sensitivity = tire->speed_sensitivity;
            contact.m_load_sensitivity = tire->load_sensitivity;
            contact.m_lat_tread_stiffness = tire->lat_tread_stiffness;
            contact.m_lon_tread_stiffness = tire->lon_tread_stiffness;
            
            registry.assign<constraint>(manifold_entity, contact);

            // Swap relation to ensure the cylinder is in the first entity.
            auto &shapeB = registry.get<shape>(rel.entity[1]);
            if (std::holds_alternative<cylinder_shape>(shapeB.var)) {
                std::swap(rel.entity[0], rel.entity[1]);
            }
        }
    } else {
        // For regular contacts one `contact_constraint` is assigned to every 
        // new point.
        auto contact = contact_constraint();
        contact.stiffness = stiffness;
        contact.damping = damping;

        // A new and identical relation is created for every contact point
        // because every constraint needs a corresponding relation to refer
        // to the constrained entities. 
        // Referring to `rel.entity` below is not safe because a new relation
        // is being created. Thus, make a copy of its entity array. 
        auto rel_entity = rel.entity;
        registry.assign<relation>(contact_entity, rel_entity[0], rel_entity[1]);
        auto &con = registry.assign<constraint>(contact_entity, contact);

        auto normal_row_entity = registry.create();
        con.row[con.num_rows++] = normal_row_entity;
        auto friction_row_entity = registry.create();
        con.row[con.num_rows++] = friction_row_entity;

        // Assign row component and associate entities.
        auto &normal_row = registry.assign<constraint_row>(normal_row_entity);
        normal_row.entity = rel_entity;
        normal_row.restitution = cp.restitution;
        normal_row.priority = 0;
        auto &friction_row = registry.assign<constraint_row>(friction_row_entity);
        friction_row.entity = rel_entity;
        friction_row.priority = 1;
    }
}

static
void update_contact_constraint(entt::entity entity, entt::entity contact_entity,
                               entt::registry &registry, contact_point &cp, relation &rel) {
    auto *con = registry.try_get<constraint>(contact_entity);
    if (con && std::holds_alternative<contact_constraint>(con->var)) {
        // One of the existing contacts has been replaced by the new.
        // Update its rows.
        auto &normal_row = registry.get<constraint_row>(con->row[0]);
        auto &friction_row = registry.get<constraint_row>(con->row[1]);
        normal_row.restitution = cp.restitution;

        // Zero out warm-starting impulses.
        normal_row.impulse = 0;
        friction_row.impulse = 0;
    }
}

void narrowphase::process_collision(entt::entity entity, contact_manifold &manifold, 
                                    relation &rel, const collision_result &result) {
    auto cp_view = registry->view<contact_point>();
    // Merge new with existing contact points.
    for (size_t i = 0; i < result.num_points; ++i) {
        auto &rp = result.point[i];

        // Find closest existing point.
        auto shortest_dist = contact_caching_threshold * contact_caching_threshold;
        auto nearest_idx = max_contacts;

        for (size_t k = 0; k < manifold.num_points; ++k) {
            auto &cp = cp_view.get(manifold.point_entity[k]);
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
            auto &cp = cp_view.get(manifold.point_entity[nearest_idx]);
            ++cp.lifetime;
            merge_point(rp, cp);
        } else {
            // Assign it to array of points and set it up.
            // Find best insertion index.
            std::array<vector3, max_contacts> pivots;
            std::array<scalar, max_contacts> distances;
            for (size_t i = 0; i < manifold.num_points; ++i) {
                auto &cp = cp_view.get(manifold.point_entity[i]);
                pivots[i] = cp.pivotA;
                distances[i] = cp.distance;
            }

            auto idx = insert_index(pivots, distances, manifold.num_points, rp.pivotA, rp.distance);

            if (idx < max_contacts) {
                auto is_new_contact = idx == manifold.num_points;

                if (is_new_contact) {
                    auto contact_entity = registry->create();
                    manifold.point_entity[idx] = contact_entity;
                    ++manifold.num_points;

                    auto &cp = registry->assign<contact_point>(
                        contact_entity, 
                        entity, // parent
                        rp.pivotA, // pivotA
                        rp.pivotB, // pivotB
                        rp.normalB, // normalB
                        0, // friction
                        0, // restitution
                        0, // lifetime
                        rp.distance // distance
                    );

                    create_contact_constraint(entity, contact_entity, *registry, cp, rel);
                } else {
                    // Replace existing contact point.
                    auto contact_entity = manifold.point_entity[idx];
                    auto &cp = cp_view.get(contact_entity);
                    cp.lifetime = 0;
                    merge_point(rp, cp);
                    update_contact_constraint(entity, contact_entity, *registry, cp, rel);
                }
            }
        }
    }
}

void narrowphase::prune(entt::entity entity, contact_manifold &manifold, const relation &rel,
                        const vector3 &posA, const quaternion &ornA, 
                        const vector3 &posB, const quaternion &ornB) {
    constexpr auto threshold_sqr = contact_breaking_threshold * contact_breaking_threshold;
    auto view = registry->view<contact_point>();
    // Remove separating contact points.
    for (size_t i = manifold.num_points; i > 0; --i) {
        size_t k = i - 1;
        auto point_entity = manifold.point_entity[k];
        auto &cp = view.get(point_entity);
        auto pA = posA + rotate(ornA, cp.pivotA);
        auto pB = posB + rotate(ornB, cp.pivotB);
        auto n = rotate(ornB, cp.normalB);
        auto d = pA - pB;
        auto dn = dot(d, n); // separation along normal
        auto dp = d - dn * n; // tangential separation on contact plane

        if (dn > contact_breaking_threshold || length_sqr(dp) > threshold_sqr) {
            registry->destroy(point_entity);

            // Swap with last element.
            size_t last_idx = manifold.num_points - 1;
            
            if (last_idx != k) {
                manifold.point_entity[k] = manifold.point_entity[last_idx];
            }

            --manifold.num_points;
        }
    }
}

}