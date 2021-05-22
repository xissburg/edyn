#include "edyn/collision/narrowphase.hpp"
#include "edyn/config/constants.hpp"
#include "edyn/constraints/constraint.hpp"
#include "edyn/comp/material.hpp"
#include "edyn/comp/position.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/comp/shape_index.hpp"
#include "edyn/comp/tag.hpp"
#include "edyn/comp/aabb.hpp"
#include "edyn/comp/dirty.hpp"
#include "edyn/comp/continuous.hpp"
#include "edyn/shapes/shapes.hpp"
#include "edyn/collision/contact_manifold.hpp"
#include "edyn/collision/contact_point.hpp"
#include "edyn/collision/collide.hpp"
#include "edyn/constraints/contact_constraint.hpp"
#include "edyn/constraints/constraint_impulse.hpp"
#include "edyn/math/geom.hpp"
#include "edyn/util/constraint_util.hpp"
#include "edyn/parallel/parallel_for_async.hpp"
#include <entt/entt.hpp>

namespace edyn {

// Update distance of persisted contact points.
static
void update_contact_distances(entt::registry &registry) {
    auto cp_view = registry.view<contact_point>();
    auto tr_view = registry.view<position, orientation>();

    cp_view.each([&] (auto, contact_point &cp) {
        auto [posA, ornA] = tr_view.get<position, orientation>(cp.body[0]);
        auto [posB, ornB] = tr_view.get<position, orientation>(cp.body[1]);
        auto pivotA_world = posA + rotate(ornA, cp.pivotA);
        auto pivotB_world = posB + rotate(ornB, cp.pivotB);
        auto normal_world = rotate(ornB, cp.normalB);
        cp.distance = dot(normal_world, pivotA_world - pivotB_world);
    });
}

// Merges a `collision_point` onto a `contact_point`.
static
void merge_point(const collision_result::collision_point &rp, contact_point &cp) {
    cp.pivotA = rp.pivotA;
    cp.pivotB = rp.pivotB;
    cp.normalB = rp.normalB;
    cp.distance = rp.distance;
}

static
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

    // Contact constraints are never graph edges since they're effectively
    // a child of a manifold and the manifold is the graph edge.
    constexpr auto is_graph_edge = false;
    auto &contact = make_constraint<contact_constraint>(contact_entity, registry, cp.body[0], cp.body[1], is_graph_edge);
    contact.stiffness = stiffness;
    contact.damping = damping;
}

static
size_t find_nearest_contact(const contact_point &cp,
                            const collision_result &result) {
    auto shortest_dist = contact_caching_threshold * contact_caching_threshold;
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

static
void create_contact_point(entt::registry& registry,
                          entt::entity manifold_entity,
                          contact_manifold& manifold,
                          const collision_result::collision_point& rp) {
    auto idx = manifold.num_points();
    if (idx >= max_contacts) return;

    auto contact_entity = registry.create();
    manifold.point[idx] = contact_entity;

    auto& cp = registry.emplace<contact_point>(
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

    if (registry.has<material>(manifold.body[0]) && registry.has<material>(manifold.body[1])) {
        create_contact_constraint(registry, contact_entity, cp);
    }

    auto &contact_dirty = registry.get_or_emplace<dirty>(contact_entity);
    contact_dirty.set_new().created<contact_point>();

    if (registry.has<continuous_contacts_tag>(manifold.body[0]) ||
        registry.has<continuous_contacts_tag>(manifold.body[1])) {

        registry.emplace<edyn::continuous>(contact_entity).insert<edyn::contact_point>();
        contact_dirty.created<continuous>();
    }

    registry.get_or_emplace<dirty>(manifold_entity).updated<contact_manifold>();
}

static
void destroy_contact_point(entt::registry &registry, entt::entity manifold_entity, entt::entity contact_entity) {
    registry.destroy(contact_entity);
    registry.get_or_emplace<dirty>(manifold_entity).updated<contact_manifold>();
}

static
bool maybe_remove_point(contact_manifold &manifold, const contact_point &cp, size_t pt_idx,
                        const vector3 &posA, const quaternion &ornA,
                        const vector3 &posB, const quaternion &ornB) {
    constexpr auto threshold_sqr = contact_breaking_threshold * contact_breaking_threshold;

    // Remove separating contact points.
    auto pA = posA + rotate(ornA, cp.pivotA);
    auto pB = posB + rotate(ornB, cp.pivotB);
    auto n = rotate(ornB, cp.normalB);
    auto d = pA - pB;
    auto normal_dist = dot(d, n);
    auto tangential_dir = d - normal_dist * n; // tangential separation on contact plane
    auto tangential_dist_sqr = length_sqr(tangential_dir);

    if (normal_dist < contact_breaking_threshold &&
        tangential_dist_sqr < threshold_sqr) {
        return false;
    }

    // Swap with last element.
    size_t last_idx = manifold.num_points() - 1;

    if (last_idx != pt_idx) {
        manifold.point[pt_idx] = manifold.point[last_idx];
    }

    manifold.point[last_idx] = entt::null;

    return true;
}

using contact_point_view_t = entt::basic_view<entt::entity, entt::exclude_t<>, contact_point, constraint_impulse>;

template<typename NewPointFunc, typename DestroyPointFunc> static
void process_collision(entt::entity manifold_entity, contact_manifold &manifold,
                       const collision_result &result,
                       const contact_point_view_t &cp_view,
                       const transform_view_t &tr_view,
                       NewPointFunc new_point_func,
                       DestroyPointFunc destroy_point_func) {
    auto [posA, ornA] = tr_view.get<position, orientation>(manifold.body[0]);
    auto [posB, ornB] = tr_view.get<position, orientation>(manifold.body[1]);

    // Merge new with existing contact points.
    auto processed_indices = std::array<bool, max_contacts>{};
    std::fill(processed_indices.begin(), processed_indices.end(), false);

    for (size_t i = manifold.num_points(); i > 0; --i) {
        auto pt_idx = i - 1;
        // Find a point in the result that's closest to the current point and
        // replace it. If there isn't any, check if the point is separating and
        // remove it if so. Increment lifetime if the point survives or gets
        // replaced by a matching result point.
        auto point_entity = manifold.point[pt_idx];
        auto &cp = cp_view.get<contact_point>(point_entity);
        ++cp.lifetime;

        auto nearest_idx = find_nearest_contact(cp, result);

        if (nearest_idx < result.num_points) {
            merge_point(result.point[nearest_idx], cp);
            processed_indices[nearest_idx] = true;
        } else if (maybe_remove_point(manifold, cp, pt_idx, posA, ornA, posB, ornB)) {
            destroy_point_func(point_entity);
        }
    }

    // Insert the remaining points seeking to maximize the contact area.
    for (size_t pt_idx = 0; pt_idx < result.num_points; ++pt_idx) {
        if (processed_indices[pt_idx]) {
            continue;
        }

        auto &rp = result.point[pt_idx];
        auto insert_idx = manifold.num_points();

        if (insert_idx == max_contacts) {
            // Look for a point to be replaced. Try pivotA first.
            std::array<vector3, max_contacts> pivots;
            std::array<scalar, max_contacts> distances;
            auto num_points = manifold.num_points();

            for (size_t i = 0; i < num_points; ++i) {
                auto &cp = cp_view.get<contact_point>(manifold.point[i]);
                pivots[i] = cp.pivotB;
                distances[i] = cp.distance;
            }

            insert_idx = insert_index(pivots, distances, num_points, rp.pivotB, rp.distance);

            // No closest point found for pivotA, try pivotB.
            if (insert_idx >= num_points) {
                for (size_t i = 0; i < num_points; ++i) {
                    auto &cp = cp_view.get<contact_point>(manifold.point[i]);
                    pivots[i] = cp.pivotB;
                }

                insert_idx = insert_index(pivots, distances, manifold.num_points(), rp.pivotB, rp.distance);
            }
        }

        if (insert_idx < max_contacts) {
            auto is_new_contact = insert_idx == manifold.num_points();

            if (is_new_contact) {
                new_point_func(rp);
            } else {
                // Replace existing contact point.
                auto contact_entity = manifold.point[insert_idx];
                auto &cp = cp_view.get<contact_point>(contact_entity);
                cp.lifetime = 0;
                merge_point(rp, cp);

                // Zero out warm-starting impulses.
                auto &imp = cp_view.get<constraint_impulse>(contact_entity);
                imp.zero_out();
            }
        }
    }
}

void detect_collision(const contact_manifold &manifold, collision_result &result,
                      const body_view_t &body_view, const tuple_of_shape_views_t &views_tuple) {
    auto [aabbA, posA, ornA] = body_view.get<AABB, position, orientation>(manifold.body[0]);
    auto [aabbB, posB, ornB] = body_view.get<AABB, position, orientation>(manifold.body[1]);
    const auto offset = vector3_one * -contact_breaking_threshold;

    // Only proceed to closest points calculation if AABBs intersect, since
    // a manifold is allowed to exist whilst the AABB separation is smaller
    // than `manifold.separation_threshold` which is greater than the
    // contact breaking threshold.
    if (intersect(aabbA.inset(offset), aabbB)) {
        auto shape_indexA = body_view.get<shape_index>(manifold.body[0]);
        auto shape_indexB = body_view.get<shape_index>(manifold.body[1]);
        auto ctx = collision_context{posA, ornA, aabbA, posB, ornB, aabbB, contact_breaking_threshold};

        visit_shape(shape_indexA, manifold.body[0], views_tuple, [&] (auto &&shA) {
            visit_shape(shape_indexB, manifold.body[1], views_tuple, [&] (auto &&shB) {
                collide(shA, shB, ctx, result);
            });
        });
    } else {
        result.num_points = 0;
    }
}

void process_result(entt::registry &registry, entt::entity manifold_entity,
                    contact_manifold &manifold, const collision_result &result,
                    const transform_view_t &tr_view) {
    auto cp_view = registry.view<contact_point, constraint_impulse>();

    process_collision(manifold_entity, manifold, result, cp_view, tr_view,
                      [&] (const collision_result::collision_point &rp) {
        create_contact_point(registry, manifold_entity, manifold, rp);
    }, [&] (entt::entity contact_entity) {
        destroy_contact_point(registry, manifold_entity, contact_entity);
    });
}

narrowphase::narrowphase(entt::registry &reg)
    : m_registry(&reg)
{}

bool narrowphase::parallelizable() const {
    return m_registry->size<contact_manifold>() > 1;
}

void narrowphase::update() {
    update_contact_distances(*m_registry);

    auto manifold_view = m_registry->view<contact_manifold>();
    update_contact_manifolds(manifold_view.begin(), manifold_view.end(), manifold_view);
}

void narrowphase::update_async(job &completion_job) {
    update_contact_distances(*m_registry);

    EDYN_ASSERT(parallelizable());

    auto manifold_view = m_registry->view<contact_manifold>();
    auto body_view = m_registry->view<AABB, shape_index, position, orientation>();
    auto tr_view = m_registry->view<position, orientation>();
    auto cp_view = m_registry->view<contact_point, constraint_impulse>();
    auto shapes_views_tuple = get_tuple_of_shape_views(*m_registry);

    // Resize result collection vectors to allocate one slot for each iteration
    // of the parallel_for.
    m_cp_construction_infos.resize(manifold_view.size());
    m_cp_destruction_infos.resize(manifold_view.size());
    auto &dispatcher = job_dispatcher::global();

    parallel_for_async(dispatcher, size_t{0}, manifold_view.size(), size_t{1}, completion_job,
            [this, body_view, tr_view, manifold_view, cp_view, shapes_views_tuple] (size_t index) {
        auto entity = manifold_view[index];
        auto &manifold = manifold_view.get(entity);
        collision_result result;
        auto &construction_info = m_cp_construction_infos[index];
        auto &destruction_info = m_cp_destruction_infos[index];

        detect_collision(manifold, result, body_view, shapes_views_tuple);
        process_collision(entity, manifold, result, cp_view, tr_view,
                          [&construction_info] (const collision_result::collision_point &rp) {
            construction_info.point[construction_info.count++] = rp;
        }, [&destruction_info] (entt::entity contact_entity) {
            destruction_info.contact_entity[destruction_info.count++] = contact_entity;
        });
    });
}

void narrowphase::finish_async_update() {
    auto manifold_view = m_registry->view<contact_manifold>();

    // Create contact points.
    for (size_t i = 0; i < manifold_view.size(); ++i) {
        auto entity = manifold_view[i];
        auto &manifold = manifold_view.get(entity);
        auto &info_result = m_cp_construction_infos[i];

        for (size_t j = 0; j < info_result.count; ++j) {
            create_contact_point(*m_registry, entity, manifold, info_result.point[j]);
        }
    }

    m_cp_construction_infos.clear();

    // Destroy contact points.
    for (size_t i = 0; i < manifold_view.size(); ++i) {
        auto entity = manifold_view[i];
        auto &info_result = m_cp_destruction_infos[i];

        for (size_t j = 0; j < info_result.count; ++j) {
            destroy_contact_point(*m_registry, entity, info_result.contact_entity[j]);
        }
    }

    m_cp_destruction_infos.clear();
}

}
