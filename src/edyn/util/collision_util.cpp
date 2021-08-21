#include "edyn/util/collision_util.hpp"
#include "edyn/comp/material.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/math/quaternion.hpp"
#include "edyn/util/constraint_util.hpp"
#include "edyn/constraints/contact_constraint.hpp"
#include "edyn/collision/collide.hpp"
#include "edyn/comp/continuous.hpp"
#include "edyn/comp/tag.hpp"
#include "edyn/math/math.hpp"
#include "edyn/dynamics/material_mixing.hpp"

namespace edyn {

void update_contact_distances(entt::registry &registry) {
    auto cp_view = registry.view<contact_point>();
    auto tr_view = registry.view<position, orientation>();
    auto com_view = registry.view<center_of_mass>();

    cp_view.each([&] (contact_point &cp) {
        auto [posA, ornA] = tr_view.get<position, orientation>(cp.body[0]);
        auto [posB, ornB] = tr_view.get<position, orientation>(cp.body[1]);
        auto originA = static_cast<vector3>(posA);
        auto originB = static_cast<vector3>(posB);

        if (com_view.contains(cp.body[0])) {
            auto &com = com_view.get(cp.body[0]);
            originA = to_world_space(-com, posA, ornA);
        }

        if (com_view.contains(cp.body[1])) {
            auto &com = com_view.get(cp.body[1]);
            originB = to_world_space(-com, posB, ornB);
        }

        auto pivotA_world = to_world_space(cp.pivotA, originA, ornA);
        auto pivotB_world = to_world_space(cp.pivotB, originB, ornB);
        cp.distance = dot(cp.normal, pivotA_world - pivotB_world);
    });
}

void merge_point(const collision_result::collision_point &rp, contact_point &cp) {
    cp.pivotA = rp.pivotA;
    cp.pivotB = rp.pivotB;
    cp.normal = rp.normal;
    cp.distance = rp.distance;
}

void create_contact_constraint(entt::registry &registry,
                               entt::entity contact_entity,
                               contact_point &cp) {
    auto &materialA = registry.get<material>(cp.body[0]);
    auto &materialB = registry.get<material>(cp.body[1]);

    auto &material_table = registry.ctx<material_mix_table>();
    auto stiffness = large_scalar;
    auto damping = large_scalar;

    if (auto *material = material_table.try_get({materialA.id, materialB.id})) {
        cp.restitution = material->restitution;
        cp.friction = material->friction;
        cp.roll_friction = material->roll_friction;
        cp.spin_friction = material->spin_friction;
        stiffness = material->stiffness;
        damping = material->damping;
    } else {
        cp.restitution = material_mix_restitution(materialA.restitution, materialB.restitution);
        cp.friction = material_mix_friction(materialA.friction, materialB.friction);
        cp.roll_friction = material_mix_roll_friction(materialA.roll_friction, materialB.roll_friction);
        cp.spin_friction = material_mix_spin_friction(materialA.spin_friction, materialB.spin_friction);

        if (materialA.stiffness < large_scalar || materialB.stiffness < large_scalar) {
            stiffness = material_mix_stiffness(materialA.stiffness, materialB.stiffness);
            damping = material_mix_damping(materialA.damping, materialB.damping);
        }
    }

    // Contact constraints are never graph edges since they're effectively
    // a child of a manifold and the manifold is the graph edge.
    constexpr auto is_graph_edge = false;
    auto &contact = make_constraint<contact_constraint>(contact_entity, registry, cp.body[0], cp.body[1], is_graph_edge);
    contact.stiffness = stiffness;
    contact.damping = damping;
}

size_t find_nearest_contact(const contact_point &cp,
                            const collision_result &result) {
    auto shortest_dist_sqr = square(contact_caching_threshold);
    auto nearest_idx = result.num_points;

    for (size_t i = 0; i < result.num_points; ++i) {
        auto &coll_pt = result.point[i];
        auto dA = length_sqr(coll_pt.pivotA - cp.pivotA);
        auto dB = length_sqr(coll_pt.pivotB - cp.pivotB);

        if (dA < shortest_dist_sqr) {
            shortest_dist_sqr = dA;
            nearest_idx = i;
        }

        if (dB < shortest_dist_sqr) {
            shortest_dist_sqr = dB;
            nearest_idx = i;
        }
    }

    return nearest_idx;
}

size_t find_nearest_contact_rolling(const collision_result &result, const vector3 &cp_pivot,
                                    const vector3 &origin, const quaternion &orn,
                                    const vector3 &angvel, scalar dt) {
    // Calculate previous orientation by integrating the angular velocity
    // backwards and check if the contact point pivot lies near the same
    // location as the result point in world space.
    auto nearest_idx = result.num_points;
    auto prev_orn = integrate(orn, angvel, -dt);
    auto prev_pivot = to_world_space(cp_pivot, origin, prev_orn);
    auto shortest_dist_sqr = square(contact_caching_threshold);

    for (size_t i = 0; i < result.num_points; ++i) {
        auto &coll_pt = result.point[i];
        auto pivotA = to_world_space(coll_pt.pivotA, origin, orn);
        auto dist_sqr = distance_sqr(pivotA, prev_pivot);

        if (dist_sqr < shortest_dist_sqr) {
            shortest_dist_sqr = dist_sqr;
            nearest_idx = i;
        }
    }

    return nearest_idx;
}

entt::entity create_contact_point(entt::registry& registry,
                                entt::entity manifold_entity,
                                contact_manifold& manifold,
                                const collision_result::collision_point& rp) {
    auto idx = manifold.num_points();

    EDYN_ASSERT(idx < max_contacts);

    auto contact_entity = registry.create();
    manifold.point[idx] = contact_entity;

    auto local_normal = vector3{};

    if (rp.normal_attachment != contact_normal_attachment::none) {
        auto idx = rp.normal_attachment == contact_normal_attachment::normal_on_A ? 0 : 1;
        auto &orn = registry.get<orientation>(manifold.body[idx]);
        local_normal = rotate(conjugate(orn), rp.normal);
    }

    registry.emplace<contact_point>(
        contact_entity,
        manifold.body,
        rp.pivotA, // pivotA
        rp.pivotB, // pivotB
        rp.normal, // world space normal
        local_normal, // object space normal
        rp.normal_attachment, // to which rigid body the local normal is attached
        scalar{}, // friction
        scalar{}, // spin friction
        scalar{}, // roll friction
        scalar{}, // restitution
        uint32_t{0}, // lifetime
        rp.distance // distance
    );

    auto &contact_dirty = registry.get_or_emplace<dirty>(contact_entity);
    contact_dirty.set_new().created<contact_point>();

    if (registry.has<continuous_contacts_tag>(manifold.body[0]) ||
        registry.has<continuous_contacts_tag>(manifold.body[1])) {

        registry.emplace<edyn::continuous>(contact_entity).insert<edyn::contact_point>();
        contact_dirty.created<continuous>();
    }

    registry.get_or_emplace<dirty>(manifold_entity).updated<contact_manifold>();

    return contact_entity;
}

bool maybe_remove_point(contact_manifold &manifold, const contact_point &cp, size_t pt_idx,
                        const vector3 &posA, const quaternion &ornA,
                        const vector3 &posB, const quaternion &ornB) {
    constexpr auto threshold = contact_breaking_threshold;
    constexpr auto threshold_sqr = threshold * threshold;

    // Remove separating contact points.
    auto pA = to_world_space(cp.pivotA, posA, ornA);
    auto pB = to_world_space(cp.pivotB, posB, ornB);
    auto n = cp.normal;
    auto d = pA - pB;
    auto normal_dist = dot(d, n);
    auto tangential_dir = d - normal_dist * n; // tangential separation on contact plane
    auto tangential_dist_sqr = length_sqr(tangential_dir);

    if (normal_dist < threshold && tangential_dist_sqr < threshold_sqr) {
        return false;
    }

    // Swap with last element.
    EDYN_ASSERT(manifold.num_points() > 0);
    size_t last_idx = manifold.num_points() - 1;
    manifold.point[pt_idx] = manifold.point[last_idx];
    manifold.point[last_idx] = entt::null;

    return true;
}

void destroy_contact_point(entt::registry &registry, entt::entity manifold_entity, entt::entity contact_entity) {
    registry.destroy(contact_entity);
    registry.get_or_emplace<dirty>(manifold_entity).updated<contact_manifold>();
}

void detect_collision(std::array<entt::entity, 2> body, collision_result &result,
                      const detect_collision_body_view_t &body_view, const com_view_t &com_view,
                      const tuple_of_shape_views_t &views_tuple) {
    auto [aabbA, posA, ornA] = body_view.get<AABB, position, orientation>(body[0]);
    auto [aabbB, posB, ornB] = body_view.get<AABB, position, orientation>(body[1]);
    const auto offset = vector3_one * -contact_breaking_threshold;

    // Only proceed to closest points calculation if AABBs intersect, since
    // a manifold is allowed to exist whilst the AABB separation is smaller
    // than `manifold.separation_threshold` which is greater than the
    // contact breaking threshold.
    if (intersect(aabbA.inset(offset), aabbB)) {
        auto originA = static_cast<vector3>(posA);
        auto originB = static_cast<vector3>(posB);

        if (com_view.contains(body[0])) {
            auto &com = com_view.get(body[0]);
            originA = to_world_space(-com, posA, ornA);
        }

        if (com_view.contains(body[1])) {
            auto &com = com_view.get(body[1]);
            originB = to_world_space(-com, posB, ornB);
        }

        auto shape_indexA = body_view.get<shape_index>(body[0]);
        auto shape_indexB = body_view.get<shape_index>(body[1]);
        auto ctx = collision_context{originA, ornA, aabbA, originB, ornB, aabbB, collision_threshold};

        visit_shape(shape_indexA, body[0], views_tuple, [&] (auto &&shA) {
            visit_shape(shape_indexB, body[1], views_tuple, [&] (auto &&shB) {
                collide(shA, shB, ctx, result);
            });
        });
    } else {
        result.num_points = 0;
    }
}

}
