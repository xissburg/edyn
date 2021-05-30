#ifndef EDYN_UTIL_COLLISION_UTIL_HPP
#define EDYN_UTIL_COLLISION_UTIL_HPP

#include <algorithm>
#include <entt/fwd.hpp>
#include "edyn/comp/aabb.hpp"
#include "edyn/comp/position.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/shapes/shapes.hpp"
#include "edyn/collision/contact_point.hpp"
#include "edyn/collision/contact_manifold.hpp"
#include "edyn/collision/collision_result.hpp"
#include "edyn/constraints/constraint_impulse.hpp"

namespace edyn {

/**
 * Update distance of persisted contact points.
 */
void update_contact_distances(entt::registry &registry);

/**
 * Merges a `collision_point` onto a `contact_point`.
 */
void merge_point(const collision_result::collision_point &rp, contact_point &cp);

/**
 * Creates a contact constraint for a contact point.
 */
void create_contact_constraint(entt::registry &registry,
                               entt::entity contact_entity,
                               contact_point &cp);

/**
 * Finds the index of a point in the result that's closest to the contact point.
 * This is used to find a similar point in the result which could be replaced
 * by the new contact point.
 */
size_t find_nearest_contact(const contact_point &cp,
                            const collision_result &result);

/**
 * Creates a contact point from a result point and inserts it into a
 * manifold.
 */
entt::entity create_contact_point(entt::registry& registry,
                                  entt::entity manifold_entity,
                                  contact_manifold& manifold,
                                  const collision_result::collision_point& rp);

/**
 * Removes a contact point from a manifold if it's separating.
 */
bool maybe_remove_point(contact_manifold &manifold, const contact_point &cp, size_t pt_idx,
                        const vector3 &posA, const quaternion &ornA,
                        const vector3 &posB, const quaternion &ornB);

/**
 * Destroys a contact point that has been removed from the manifold
 * using `maybe_remove_point`.
 */
void destroy_contact_point(entt::registry &registry, entt::entity manifold_entity,
                           entt::entity contact_entity);

using detect_collision_body_view_t = entt::basic_view<entt::entity, entt::exclude_t<>,
                                     AABB, shape_index, position, orientation>;

/**
 * Detects collision between two bodies and adds closest points to the given
 * collision result
 */
void detect_collision(std::array<entt::entity, 2> body, collision_result &,
                      const detect_collision_body_view_t &, const tuple_of_shape_views_t &);

/**
 * Processes a collision result and inserts/replaces points into the manifold.
 * It also removes points in the manifold that are separating. `new_point_func`
 * is called for each point that is created and `destroy_point_func` is called
 * for every point that is removed (remember to call `destroy_contact_point`
 * when appropriate for each point that is removed).
 */
template<typename ContactPointView, typename ImpulseView, typename TransformView,
         typename NewPointFunc, typename DestroyPointFunc>
void process_collision(entt::entity manifold_entity, contact_manifold &manifold,
                       const collision_result &result,
                       ContactPointView &cp_view,
                       ImpulseView &imp_view,
                       TransformView &tr_view,
                       NewPointFunc new_point_func,
                       DestroyPointFunc destroy_point_func) {
    auto [posA, ornA] = tr_view.template get<position, orientation>(manifold.body[0]);
    auto [posB, ornB] = tr_view.template get<position, orientation>(manifold.body[1]);

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
        auto &cp = cp_view.template get<contact_point>(point_entity);
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
                auto &cp = cp_view.template get<contact_point>(manifold.point[i]);
                pivots[i] = cp.pivotB;
                distances[i] = cp.distance;
            }

            insert_idx = insert_index(pivots, distances, num_points, rp.pivotB, rp.distance);

            // No closest point found for pivotA, try pivotB.
            if (insert_idx >= num_points) {
                for (size_t i = 0; i < num_points; ++i) {
                    auto &cp = cp_view.template get<contact_point>(manifold.point[i]);
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
                auto &cp = cp_view.template get<contact_point>(contact_entity);
                cp.lifetime = 0;
                merge_point(rp, cp);

                // Zero out warm-starting impulses. Check if it contains a `constraint_impulse`
                // because in a rare occasion it might be replacing a new contact point.
                if (imp_view.contains(contact_entity)) {
                    auto &imp = imp_view.template get<constraint_impulse>(contact_entity);
                    imp.zero_out();
                }
            }
        }
    }
}

}

#endif // EDYN_UTIL_COLLISION_UTIL_HPP
