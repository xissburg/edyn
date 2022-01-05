#ifndef EDYN_UTIL_COLLISION_UTIL_HPP
#define EDYN_UTIL_COLLISION_UTIL_HPP

#include <algorithm>
#include <entt/entity/fwd.hpp>
#include <entt/entity/entity.hpp>
#include "edyn/comp/aabb.hpp"
#include "edyn/comp/material.hpp"
#include "edyn/comp/position.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/comp/origin.hpp"
#include "edyn/comp/angvel.hpp"
#include "edyn/shapes/shapes.hpp"
#include "edyn/collision/contact_point.hpp"
#include "edyn/collision/contact_manifold.hpp"
#include "edyn/collision/contact_manifold_events.hpp"
#include "edyn/collision/collision_result.hpp"

namespace edyn {

/**
 * Update distance of persisted contact points.
 */
void update_contact_distances(entt::registry &registry);

using orientation_view_t = entt::basic_view<entt::entity, entt::exclude_t<>, orientation>;
using material_view_t = entt::basic_view<entt::entity, entt::exclude_t<>, material>;
using mesh_shape_view_t = entt::basic_view<entt::entity, entt::exclude_t<>, mesh_shape>;
using paged_mesh_shape_view_t = entt::basic_view<entt::entity, entt::exclude_t<>, paged_mesh_shape>;

/**
 * Merges a `collision_point` onto a `contact_point`. It needs the material and
 * mesh shape views in order to update the contact point material properties
 * in case a mesh shape with per-vertex materials is involved.
 */
void merge_point(std::array<entt::entity, 2> body,
                 const collision_result::collision_point &rp, contact_point &cp,
                 const orientation_view_t &, const material_view_t &,
                 const mesh_shape_view_t &, const paged_mesh_shape_view_t &);

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
 * Special version of `find_nearest_contact` to find similar points for rolling
 * objects.
 */
size_t find_nearest_contact_rolling(const collision_result &result, const vector3 &cp_pivot,
                                    const vector3 &origin, const quaternion &orn,
                                    const vector3 &angvel, scalar dt);

/**
 * Creates a contact point from a result point and inserts it into a
 * manifold. The contact is inserted at the index assigned to the last
 * element of the `manifold.ids array`, i.e.
 * `manifold.point[manifold.ids[manifold.num_points-1]]`.
 */
void create_contact_point(entt::registry& registry,
                          entt::entity manifold_entity,
                          contact_manifold &manifold,
                          const collision_result::collision_point& rp);

/**
 * Removes a contact point from a manifold if it's separating.
 */
bool maybe_remove_point(contact_manifold &manifold,
                        contact_manifold_events &events,
                        size_t pt_idx,
                        const vector3 &posA, const quaternion &ornA,
                        const vector3 &posB, const quaternion &ornB);

/**
 * Destroys a contact point that has been removed from the manifold
 * using `maybe_remove_point`.
 */
void destroy_contact_point(entt::registry &registry, entt::entity manifold_entity,
                           contact_manifold::contact_id_type pt_id);

using detect_collision_body_view_t = entt::basic_view<entt::entity, entt::exclude_t<>,
                                     AABB, shape_index, position, orientation>;

using origin_view_t = entt::basic_view<entt::entity, entt::exclude_t<>, origin>;

/**
 * Detects collision between two bodies and adds closest points to the given
 * collision result
 */
void detect_collision(std::array<entt::entity, 2> body, collision_result &,
                      const detect_collision_body_view_t &, const origin_view_t &,
                      const tuple_of_shape_views_t &);

/**
 * Processes a collision result and inserts/replaces points into the manifold.
 * It also removes points in the manifold that are separating. `new_point_func`
 * is called for each point that is created and `destroy_point_func` is called
 * for every point that is removed (remember to call `destroy_contact_point`
 * when appropriate for each point that is removed).
 */
template<typename TransformView, typename VelView, typename RollingView,
         typename NewPointFunc, typename DestroyPointFunc>
void process_collision(entt::entity manifold_entity,
                       contact_manifold &manifold,
                       contact_manifold_events &events,
                       const collision_result &result,
                       TransformView &tr_view,
                       VelView &vel_view,
                       RollingView &rolling_view,
                       const origin_view_t &origin_view,
                       const orientation_view_t &orn_view,
                       const material_view_t &material_view,
                       const mesh_shape_view_t &mesh_shape_view,
                       const paged_mesh_shape_view_t &paged_mesh_shape_view,
                       scalar dt,
                       NewPointFunc new_point_func,
                       DestroyPointFunc destroy_point_func) {
    auto [posA, ornA] = tr_view.template get<position, orientation>(manifold.body[0]);
    auto [posB, ornB] = tr_view.template get<position, orientation>(manifold.body[1]);

    auto originA = origin_view.contains(manifold.body[0]) ? origin_view.get<origin>(manifold.body[0]) : static_cast<vector3>(posA);
    auto originB = origin_view.contains(manifold.body[1]) ? origin_view.get<origin>(manifold.body[1]) : static_cast<vector3>(posB);

    auto rollingA = rolling_view.contains(manifold.body[0]);
    auto rollingB = rolling_view.contains(manifold.body[1]);

    // Merge new with existing contact points.
    auto merged_indices = std::array<bool, max_contacts>{};
    std::fill(merged_indices.begin(), merged_indices.end(), false);

    for (auto i = manifold.num_points; i > 0; --i) {
        auto pt_idx = i - 1;
        // Find a point in the result that's closest to the current point and
        // replace it. If there isn't any, check if the point is separating and
        // remove it if so. Increment lifetime if the point survives or gets
        // replaced by a matching result point.
        auto pt_id = manifold.ids[pt_idx];
        auto &cp = manifold.point[pt_id];
        ++cp.lifetime;

        auto nearest_idx = find_nearest_contact(cp, result);

        // Try finding a nearby point for rolling objects.
        if (nearest_idx == result.num_points && rollingA) {
            auto &angvelA = vel_view.template get<angvel>(manifold.body[0]);
            nearest_idx = find_nearest_contact_rolling(result, cp.pivotA, originA, ornA, angvelA, dt);
        }

        if (nearest_idx == result.num_points && rollingB) {
            auto &angvelB = vel_view.template get<angvel>(manifold.body[1]);
            nearest_idx = find_nearest_contact_rolling(result, cp.pivotB, originB, ornB, angvelB, dt);
        }

        if (nearest_idx < result.num_points && !merged_indices[nearest_idx]) {
            merge_point(manifold.body, result.point[nearest_idx], cp, orn_view, material_view, mesh_shape_view, paged_mesh_shape_view);
            merged_indices[nearest_idx] = true;
        } else if (maybe_remove_point(manifold, events, pt_idx, originA, ornA, originB, ornB)) {
            destroy_point_func(pt_id);
        }
    }

    // Do not continue if all result points were merged.
    auto merged_indices_end = merged_indices.begin() + result.num_points;
    if (std::find(merged_indices.begin(), merged_indices_end, false) == merged_indices_end) {
        return;
    }

    // Use a local array of points to store the final combination of existing
    // with new contact points. Points can be inserted and replaced thus doing
    // this directly into the manifold would create some confusion.
    struct local_contact_point {
        collision_result::collision_point point;
        unsigned pt_id {contact_manifold::invalid_id};
        point_insertion_type type {point_insertion_type::none};
    };

    // Start with current manifold points.
    auto local_points = std::array<local_contact_point, max_contacts>{};
    auto num_points = size_t{manifold.num_points};

    if (num_points > 0) {
        for (size_t i = 0; i < num_points; ++i) {
            auto pt_id = manifold.ids[i];
            auto &cp = manifold.point[pt_id];
            local_points[i].point = {cp.pivotA, cp.pivotB, cp.normal, cp.distance};
            local_points[i].pt_id = pt_id;
        }
    } else {
        ++num_points;
        local_points[0].point = result.point[0];
        local_points[0].type = point_insertion_type::append;
        merged_indices[0] = true;
    }

    // Insert the remaining result points seeking to maximize the contact area.
    for (size_t pt_idx = 0; pt_idx < result.num_points; ++pt_idx) {
        if (merged_indices[pt_idx]) {
            continue;
        }

        auto &rp = result.point[pt_idx];

        // Look for a point to be replaced. Try pivotA first.
        // Note that `insertion_point_index` will increment `num_points`
        // for the caller if it determines this is a new point that must be
        // appended, i.e. `res.type` will be `point_insertion_type::append`.
        std::array<vector3, max_contacts> pivots;
        std::array<scalar, max_contacts> distances;

        for (size_t i = 0; i < num_points; ++i) {
            pivots[i] = local_points[i].point.pivotA;
            distances[i] = local_points[i].point.distance;
        }

        auto res = insertion_point_index(pivots, distances, num_points, rp.pivotA, rp.distance);

        // No closest point found for pivotA, try pivotB.
        if (res.type == point_insertion_type::none) {
            for (size_t i = 0; i < num_points; ++i) {
                pivots[i] = local_points[i].point.pivotB;
            }

            res = insertion_point_index(pivots, distances, num_points, rp.pivotB, rp.distance);
        }

        if (res.type != point_insertion_type::none) {
            local_points[res.index].point = rp;
            local_points[res.index].type = res.type;
        }
    }

    // Assign some points to manifold and replace others.
    for (size_t pt_idx = 0; pt_idx < num_points; ++pt_idx) {
        auto &local_pt = local_points[pt_idx];
        switch (local_pt.type) {
        case point_insertion_type::append:
            // Notify creation of a new point if it was inserted. It could have
            // been replaced after being inserted in the steps above, but in
            // either case, it's still a new point.
            new_point_func(local_pt.point);
            break;
        case point_insertion_type::similar:
            // It was replaced by a similar point, thus merge.
            // There is the possibility that this point actually replaced a similar
            // point which is new and is not in the manifold yet, i.e. the collision
            // result has two or more points that are very close to one another.
            // Create a new point in that case.
            if (local_pt.pt_id == contact_manifold::invalid_id) {
                new_point_func(local_pt.point);
            } else {
                merge_point(manifold.body, local_pt.point, manifold.point[local_pt.pt_id],
                            orn_view, material_view, mesh_shape_view, paged_mesh_shape_view);
            }
            break;
        case point_insertion_type::replace:
            // If it was replaced, the old point must be destroyed and a new
            // point must be created.
            // There is a chance the replaced point is actually one of the new
            // and thus there's no real point to be destroyed.
            if (local_pt.pt_id != contact_manifold::invalid_id) {
                for (size_t i = 0; i < manifold.num_points; ++i) {
                    if (manifold.ids[i] == local_pt.pt_id) {
                        // Assign last to i-th and set last to null.
                        size_t last_idx = manifold.num_points - 1;
                        manifold.ids[i] = manifold.ids[last_idx];
                        manifold.ids[last_idx] = contact_manifold::invalid_id;
                        --manifold.num_points;

                        // Register contact destroyed event.
                        events.contacts_destroyed[events.num_contacts_destroyed++] = local_pt.pt_id;
                        break;
                    }
                }

                destroy_point_func(local_pt.pt_id);
            }
            new_point_func(local_pt.point);
            break;
        default:
            break;
        }
    }
}

}

#endif // EDYN_UTIL_COLLISION_UTIL_HPP
