#ifndef EDYN_UTIL_COLLISION_UTIL_HPP
#define EDYN_UTIL_COLLISION_UTIL_HPP

#include <algorithm>
#include <entt/entity/fwd.hpp>
#include <entt/entity/entity.hpp>
#include <type_traits>
#include <utility>
#include "edyn/comp/aabb.hpp"
#include "edyn/comp/material.hpp"
#include "edyn/comp/position.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/comp/origin.hpp"
#include "edyn/comp/angvel.hpp"
#include "edyn/comp/tag.hpp"
#include "edyn/comp/transient.hpp"
#include "edyn/config/config.h"
#include "edyn/config/constants.hpp"
#include "edyn/shapes/shapes.hpp"
#include "edyn/collision/contact_point.hpp"
#include "edyn/collision/contact_manifold.hpp"
#include "edyn/collision/collision_result.hpp"
#include "edyn/util/contact_manifold_util.hpp"

namespace edyn {

/**
 * Update distance of persisted contact points.
 */
void update_contact_distances(entt::registry &registry);

using orientation_view_t = entt::basic_view<entt::get_t<entt::registry::storage_for_type<orientation>>, entt::exclude_t<>>;
using material_view_t = entt::basic_view<entt::get_t<entt::registry::storage_for_type<material>>, entt::exclude_t<>>;
using mesh_shape_view_t = entt::basic_view<entt::get_t<entt::registry::storage_for_type<mesh_shape>>, entt::exclude_t<>>;
using paged_mesh_shape_view_t = entt::basic_view<entt::get_t<entt::registry::storage_for_type<paged_mesh_shape>>, entt::exclude_t<>>;

/**
 * Merges a `collision_point` onto a `contact_point`. It needs the material and
 * mesh shape views in order to update the contact point material properties
 * in case a mesh shape with per-vertex materials is involved.
 */
void merge_point(entt::registry &registry,
                 std::array<entt::entity, 2> body,
                 const collision_result::collision_point &rp,
                 contact_point &cp,
                 contact_point_material *cp_mat,
                 contact_point_geometry &cp_geom);

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

entt::entity create_contact_point(entt::registry &registry,
                                  entt::entity manifold_entity,
                                  contact_manifold &manifold,
                                  contact_manifold_state &manifold_state,
                                  const collision_result::collision_point& rp,
                                  const std::optional<transient> &transient_contact);

bool should_remove_point(const contact_point &cp,
                        const vector3 &posA, const quaternion &ornA,
                        const vector3 &posB, const quaternion &ornB);

void destroy_contact_point(entt::registry &registry, entt::entity contact_entity);

using detect_collision_body_view_t = entt::basic_view<
                                     entt::get_t<entt::registry::storage_for_type<AABB>, entt::registry::storage_for_type<shape_index>, entt::registry::storage_for_type<position>, entt::registry::storage_for_type<orientation>>,
                                     entt::exclude_t<>>;

using origin_view_t = entt::basic_view<entt::get_t<entt::registry::storage_for_type<origin>>, entt::exclude_t<>>;

/**
 * Detects collision between two bodies and adds closest points to the given
 * collision result
 */
void detect_collision(entt::registry &registry, std::array<entt::entity, 2> body, collision_result &result);

/**
 * Processes a collision result and inserts/replaces points into the manifold.
 * It also removes points in the manifold that are separating. `new_point_func`
 * is called for each point that is created and `destroy_point_func` is called
 * for every point that is removed (remember to call `destroy_contact_point`
 * when appropriate for each point that is removed).
 */
template<typename NewPointFunc, typename DestroyPointFunc>
void process_collision(entt::registry &registry, entt::entity manifold_entity,
                       const collision_result &result, scalar dt,
                       NewPointFunc new_point_func, DestroyPointFunc destroy_point_func) {
    auto tr_view = registry.view<position, orientation>();
    auto origin_view = registry.view<origin>();
    auto rolling_view = registry.view<rolling_tag>();
    auto vel_view = registry.view<angvel>();
    auto cp_view = registry.view<contact_point, contact_point_list, contact_point_geometry>();

    auto &manifold = registry.get<contact_manifold>(manifold_entity);
    auto &manifold_state = registry.get<contact_manifold_state>(manifold_entity);

    auto [posA, ornA] = tr_view.template get<position, orientation>(manifold.body[0]);
    auto [posB, ornB] = tr_view.template get<position, orientation>(manifold.body[1]);

    auto originA = origin_view.contains(manifold.body[0]) ? origin_view.get<origin>(manifold.body[0]) : static_cast<vector3>(posA);
    auto originB = origin_view.contains(manifold.body[1]) ? origin_view.get<origin>(manifold.body[1]) : static_cast<vector3>(posB);

    auto rollingA = rolling_view.contains(manifold.body[0]);
    auto rollingB = rolling_view.contains(manifold.body[1]);

    // Merge new with existing contact points.
    auto merged_indices = std::array<bool, max_contacts>{};
    std::fill(merged_indices.begin(), merged_indices.end(), false);

    entt::sparse_set points_removed;
    auto num_points = size_t{manifold_state.num_points};
    EDYN_ASSERT(manifold_state.num_points <= max_contacts);

    contact_point_for_each(cp_view, manifold_state.contact_entity, [&, &ornA=ornA, &ornB=ornB](entt::entity contact_entity) {
        // Find a point in the result that's closest to the current point and
        // replace it. If there isn't any, check if the point is separating and
        // remove it if so. Increment lifetime if the point survives or gets
        // replaced by a matching result point.
        auto &cp = cp_view.template get<contact_point>(contact_entity);
        ++cp.lifetime;

        auto nearest_idx = find_nearest_contact(cp, result);

        // Try finding a nearby point for rolling objects.
        if (nearest_idx == result.num_points && rollingA) {
            auto &angvelA = vel_view.get<angvel>(manifold.body[0]);
            nearest_idx = find_nearest_contact_rolling(result, cp.pivotA, originA, ornA, angvelA, dt);
        }

        if (nearest_idx == result.num_points && rollingB) {
            auto &angvelB = vel_view.get<angvel>(manifold.body[1]);
            nearest_idx = find_nearest_contact_rolling(result, cp.pivotB, originB, ornB, angvelB, dt);
        }

        if (nearest_idx < result.num_points && !merged_indices[nearest_idx]) {
            auto &cp_geom = cp_view.get<contact_point_geometry>(contact_entity);
            auto *cp_mat = registry.try_get<contact_point_material>(contact_entity);
            merge_point(registry, manifold.body, result.point[nearest_idx], cp, cp_mat, cp_geom);
            merged_indices[nearest_idx] = true;
        } else if (should_remove_point(cp, originA, ornA, originB, ornB)) {
            points_removed.push(contact_entity);
            EDYN_ASSERT(num_points > 0);
            --num_points;
            destroy_point_func(contact_entity);
        }
    });

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
        entt::entity contact_entity {entt::null};
        point_insertion_type type {point_insertion_type::none};
    };

    // Start with current manifold points.
    auto local_points = std::array<local_contact_point, max_contacts>{};

    if (num_points > 0) {
        auto pt_idx = 0u;
        contact_point_for_each(cp_view, manifold_state.contact_entity, [&](entt::entity contact_entity) {
            if (points_removed.contains(contact_entity)) return;
            auto [cp, cp_geom] = cp_view.template get<contact_point, contact_point_geometry>(contact_entity);
            local_points[pt_idx].point = {cp.pivotA, cp.pivotB, cp.normal, cp_geom.distance};
            local_points[pt_idx].contact_entity = contact_entity;
            ++pt_idx;
        });
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

        for (size_t i = 0; i < num_points; ++i) {
            pivots[i] = local_points[i].point.pivotA;
        }

        auto res = insertion_point_index(pivots, num_points, rp.pivotA);

        // No closest point found for pivotA, try pivotB.
        if (res.type == point_insertion_type::none) {
            for (size_t i = 0; i < num_points; ++i) {
                pivots[i] = local_points[i].point.pivotB;
            }

            res = insertion_point_index(pivots, num_points, rp.pivotB);
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
        case point_insertion_type::none:
            break;
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
            if (local_pt.contact_entity == entt::null) {
                new_point_func(local_pt.point);
            } else {
                auto [cp, cp_geom] = cp_view.get<contact_point, contact_point_geometry>(local_pt.contact_entity);
                auto *cp_mat = registry.try_get<contact_point_material>(local_pt.contact_entity);
                merge_point(registry, manifold.body, local_pt.point, cp, cp_mat, cp_geom);
            }
            break;
        case point_insertion_type::replace:
            // If it was replaced, the old point must be destroyed and a new
            // point must be created.
            // There is a chance the replaced point is actually one of the new
            // and thus there's no real point to be destroyed.
            if (local_pt.contact_entity != entt::null) {
                destroy_point_func(local_pt.contact_entity);
            }
            new_point_func(local_pt.point);
            break;
        }
    }
}

}

#endif // EDYN_UTIL_COLLISION_UTIL_HPP
