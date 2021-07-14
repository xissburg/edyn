#include "edyn/util/collision_util.hpp"
#include "edyn/comp/material.hpp"
#include "edyn/comp/tire_material.hpp"
#include "edyn/util/constraint_util.hpp"
#include "edyn/constraints/contact_constraint.hpp"
#include "edyn/constraints/contact_patch_constraint.hpp"
#include "edyn/collision/collide.hpp"
#include "edyn/comp/continuous.hpp"
#include "edyn/comp/tag.hpp"
#include "edyn/math/vector2_3_util.hpp"
#include "edyn/math/math.hpp"

namespace edyn {

void update_contact_distances(entt::registry &registry) {
    auto cp_view = registry.view<contact_point>();
    auto tr_view = registry.view<position, orientation>();

    cp_view.each([&] (contact_point &cp) {
        auto [posA, ornA] = tr_view.get<position, orientation>(cp.body[0]);
        auto [posB, ornB] = tr_view.get<position, orientation>(cp.body[1]);
        auto pivotA_world = posA + rotate(ornA, cp.pivotA);
        auto pivotB_world = posB + rotate(ornB, cp.pivotB);
        auto normal_world = rotate(ornB, cp.normalB);
        cp.distance = dot(normal_world, pivotA_world - pivotB_world);
    });
}

void merge_point(const collision_result::collision_point &rp, contact_point &cp) {
    cp.pivotA = rp.pivotA;
    cp.pivotB = rp.pivotB;
    cp.normalB = rp.normalB;
    cp.distance = rp.distance;
}

void create_contact_constraint(entt::registry &registry,
                               entt::entity contact_entity,
                               contact_point &cp) {
    auto &materialA = registry.get<material>(cp.body[0]);
    auto &materialB = registry.get<material>(cp.body[1]);

    cp.restitution = materialA.restitution * materialB.restitution;
    cp.friction = materialA.friction * materialB.friction;

    auto stiffness = large_scalar;
    auto damping = large_scalar;

    if (materialA.stiffness < large_scalar || materialB.stiffness < large_scalar) {
        stiffness = 1 / (1 / materialA.stiffness + 1 / materialB.stiffness);
        damping = 1 / (1 / materialA.damping + 1 / materialB.damping);
    }

    auto *tire0 = registry.try_get<tire_material>(cp.body[0]);
    auto *tire1 = registry.try_get<tire_material>(cp.body[1]);
    auto *tire = tire0 ? tire0 : tire1;

    // Contact constraints are never graph edges since they're effectively
    // a child of a manifold and the manifold is the graph edge.
    constexpr auto is_graph_edge = false;

    if (tire) {
        // Contact patch is always a soft contact since it needs deflection
        // to generate friction forces.
        EDYN_ASSERT(stiffness < large_scalar);

        // Swap entities to ensure the tire/cylinder is in the first entity.
        auto body0 = entt::entity{};
        auto body1 = entt::entity{};

        if (registry.has<cylinder_shape>(cp.body[0])) {
            body0 = cp.body[0];
            body1 = cp.body[1];
        } else {
            body0 = cp.body[1];
            body1 = cp.body[0];
        }

        auto &contact = make_constraint<contact_patch_constraint>(contact_entity, registry, body0, body1, is_graph_edge);
        contact.m_normal_stiffness = stiffness;
        contact.m_normal_damping = damping;
        contact.m_speed_sensitivity = tire->speed_sensitivity;
        contact.m_load_sensitivity = tire->load_sensitivity;
        contact.m_lat_tread_stiffness = tire->lat_tread_stiffness;
        contact.m_lon_tread_stiffness = tire->lon_tread_stiffness;
        initialize_contact_patch_constraint(registry, contact_entity);
    } else {
        auto &contact = make_constraint<contact_constraint>(contact_entity, registry, cp.body[0], cp.body[1], is_graph_edge);
        contact.stiffness = stiffness;
        contact.damping = damping;
    }
}

size_t find_nearest_contact(const contact_point &cp,
                            const collision_result &result) {
    auto shortest_dist = square(contact_caching_threshold);
    auto nearest_idx = result.num_points;

    for (size_t i = 0; i < result.num_points; ++i) {
        auto &coll_pt = result.point[i];
        auto dA = length_sqr(coll_pt.pivotA - cp.pivotA);
        auto dB = length_sqr(coll_pt.pivotB - cp.pivotB);

        if (dA < shortest_dist) {
            shortest_dist = dA;
            nearest_idx = i;
        }

        if (dB < shortest_dist) {
            shortest_dist = dB;
            nearest_idx = i;
        }
    }

    return nearest_idx;
}

size_t find_nearest_contact_tire(const contact_point &cp,
                                 const collision_result &result) {
    // Assuming A is the tire, calculate distance in the yz-plane, thus ignoring
    // the axial component, which means that points that are aligned along the
    // cylinder axis always get merged together.
    auto shortest_dist = square(contact_caching_threshold);
    auto nearest_idx = result.num_points;

    for (size_t i = 0; i < result.num_points; ++i) {
        auto &coll_pt = result.point[i];
        auto dist = length_sqr(to_vector2_zy(coll_pt.pivotA) - to_vector2_zy(cp.pivotA));

        if (dist < shortest_dist) {
            shortest_dist = dist;
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

    registry.emplace<contact_point>(
        contact_entity,
        manifold.body,
        rp.pivotA, // pivotA
        rp.pivotB, // pivotB
        rp.normalB, // normalB
        scalar{}, // friction
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
    auto pA = posA + rotate(ornA, cp.pivotA);
    auto pB = posB + rotate(ornB, cp.pivotB);
    auto n = rotate(ornB, cp.normalB);
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
                      const detect_collision_body_view_t &body_view, const tuple_of_shape_views_t &views_tuple) {
    auto [aabbA, posA, ornA] = body_view.get<AABB, position, orientation>(body[0]);
    auto [aabbB, posB, ornB] = body_view.get<AABB, position, orientation>(body[1]);
    const auto offset = vector3_one * -contact_breaking_threshold;

    // Only proceed to closest points calculation if AABBs intersect, since
    // a manifold is allowed to exist whilst the AABB separation is smaller
    // than `manifold.separation_threshold` which is greater than the
    // contact breaking threshold.
    if (intersect(aabbA.inset(offset), aabbB)) {
        auto shape_indexA = body_view.get<shape_index>(body[0]);
        auto shape_indexB = body_view.get<shape_index>(body[1]);
        auto ctx = collision_context{posA, ornA, aabbA, posB, ornB, aabbB, contact_breaking_threshold};

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
