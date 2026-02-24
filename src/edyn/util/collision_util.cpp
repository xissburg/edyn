#include "edyn/util/collision_util.hpp"
#include "edyn/collision/contact_manifold.hpp"
#include "edyn/collision/contact_point.hpp"
#include "edyn/comp/material.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/comp/transient.hpp"
#include "edyn/config/constants.hpp"
#include "edyn/constraints/null_constraint.hpp"
#include "edyn/math/constants.hpp"
#include "edyn/math/quaternion.hpp"
#include "edyn/math/transform.hpp"
#include "edyn/math/vector3.hpp"
#include "edyn/shapes/mesh_shape.hpp"
#include "edyn/util/constraint_util.hpp"
#include "edyn/constraints/contact_constraint.hpp"
#include "edyn/collision/collide.hpp"
#include "edyn/comp/tag.hpp"
#include "edyn/math/math.hpp"
#include "edyn/dynamics/material_mixing.hpp"
#include "edyn/math/triangle.hpp"
#include "edyn/util/island_util.hpp"
#include "edyn/util/transient_util.hpp"
#include "edyn/util/contact_manifold_util.hpp"
#include <limits>

namespace edyn {

void update_contact_distances(entt::registry &registry) {
    auto manifold_view = registry.view<const contact_manifold>(exclude_sleeping_disabled);
    auto tr_view = registry.view<position, orientation>();
    auto origin_view = registry.view<origin>();
    auto cp_view = registry.view<contact_point, contact_point_geometry, const contact_point_list>();

    for (auto [entity, cp, cp_geom, cp_list] : cp_view.each()) {
        auto [manifold] = manifold_view.get(cp_list.parent);
        auto [posA, ornA] = tr_view.get(manifold.body[0]);
        auto [posB, ornB] = tr_view.get(manifold.body[1]);
        auto originA = origin_view.contains(manifold.body[0]) ? origin_view.get<origin>(manifold.body[0]) : static_cast<vector3>(posA);
        auto originB = origin_view.contains(manifold.body[1]) ? origin_view.get<origin>(manifold.body[1]) : static_cast<vector3>(posB);

        auto pivotA_world = to_world_space(cp.pivotA, originA, ornA);
        auto pivotB_world = to_world_space(cp.pivotB, originB, ornB);
        cp_geom.distance = dot(cp.normal, pivotA_world - pivotB_world);
    }
}

static scalar get_trimesh_friction(const triangle_mesh &trimesh,
                                   const vector3 &pivot, collision_feature coll_feature) {
    auto feature = std::get<triangle_feature>(coll_feature.feature);

    switch (feature) {
    case triangle_feature::vertex:
        return trimesh.get_vertex_friction(coll_feature.index);
    case triangle_feature::edge:
        return trimesh.get_edge_friction(coll_feature.index, pivot);
    case triangle_feature::face:
        return trimesh.get_face_friction(coll_feature.index, pivot);
    }

    return {};
}

static scalar get_trimesh_restitution(const triangle_mesh &trimesh,
                                      const vector3 &pivot, collision_feature coll_feature) {
    auto feature = std::get<triangle_feature>(coll_feature.feature);

    switch (feature) {
    case triangle_feature::vertex:
        return trimesh.get_vertex_restitution(coll_feature.index);
    case triangle_feature::edge:
        return trimesh.get_edge_restitution(coll_feature.index, pivot);
    case triangle_feature::face:
        return trimesh.get_face_restitution(coll_feature.index, pivot);
    }

    return {};
}

static scalar get_paged_mesh_friction(const paged_mesh_shape &shape,
                                      const vector3 &pivot, collision_feature coll_feature) {
    auto submesh = shape.trimesh->get_submesh(coll_feature.part);

    if (submesh) {
        return get_trimesh_friction(*submesh, pivot, coll_feature);
    }

    return {};
}

static scalar get_paged_mesh_restitution(const paged_mesh_shape &shape,
                                         const vector3 &pivot, collision_feature coll_feature) {
    auto submesh = shape.trimesh->get_submesh(coll_feature.part);

    if (submesh) {
        return get_trimesh_restitution(*submesh, pivot, coll_feature);
    }

    return {};
}

static bool try_assign_per_vertex_friction(
    entt::registry &registry,
    std::array<entt::entity, 2> body,
    const contact_point &cp,
    contact_point_material &cp_mat,
    const contact_point_geometry &cp_geom) {

    auto mesh_shape_view = registry.view<mesh_shape>();
    auto paged_mesh_shape_view = registry.view<paged_mesh_shape>();
    auto material_view = registry.view<material>();

    if (mesh_shape_view.contains(body[0])) {
        auto [shapeA] = mesh_shape_view.get(body[0]);
        if (shapeA.trimesh->has_per_vertex_friction()) {
            auto [materialA] = material_view.get(body[0]);
            auto [materialB] = material_view.get(body[1]);
            auto frictionA = get_trimesh_friction(*shapeA.trimesh, cp.pivotA, *cp_geom.featureA) * materialA.friction;
            cp_mat.friction = material_mix_friction(frictionA, materialB.friction);
            return true;
        }
    } else if (mesh_shape_view.contains(body[1])) {
        auto [shapeB] = mesh_shape_view.get(body[1]);
        if (shapeB.trimesh->has_per_vertex_friction()) {
            auto [materialA] = material_view.get(body[0]);
            auto [materialB] = material_view.get(body[1]);
            auto frictionB = get_trimesh_friction(*shapeB.trimesh, cp.pivotB, *cp_geom.featureB) * materialB.friction;
            cp_mat.friction = material_mix_friction(materialA.friction, frictionB);
            return true;
        }
    } else if (paged_mesh_shape_view.contains(body[0])) {
        auto [shapeA] = paged_mesh_shape_view.get(body[0]);
        if (shapeA.trimesh->has_per_vertex_friction()) {
            auto [materialA] = material_view.get(body[0]);
            auto [materialB] = material_view.get(body[1]);
            auto frictionA = get_paged_mesh_friction(shapeA, cp.pivotA, *cp_geom.featureA) * materialA.friction;
            cp_mat.friction = material_mix_friction(frictionA, materialB.friction);
            return true;
        }
    } else if (paged_mesh_shape_view.contains(body[1])) {
        auto [shapeB] = paged_mesh_shape_view.get(body[1]);
        if (shapeB.trimesh->has_per_vertex_friction()) {
            auto [materialA] = material_view.get(body[0]);
            auto [materialB] = material_view.get(body[1]);
            auto frictionB = get_paged_mesh_friction(shapeB, cp.pivotB, *cp_geom.featureB) * materialB.friction;
            cp_mat.friction = material_mix_friction(materialA.friction, frictionB);
            return true;
        }
    }

    return false;
}

static bool try_assign_per_vertex_restitution(
    entt::registry &registry,
    std::array<entt::entity, 2> body,
    const contact_point &cp,
    contact_point_material &cp_mat,
    const contact_point_geometry &cp_geom) {

    auto mesh_shape_view = registry.view<mesh_shape>();
    auto paged_mesh_shape_view = registry.view<paged_mesh_shape>();
    auto material_view = registry.view<material>();

    if (mesh_shape_view.contains(body[0])) {
        auto [shapeA] = mesh_shape_view.get(body[0]);
        if (shapeA.trimesh->has_per_vertex_restitution()) {
            auto [materialA] = material_view.get(body[0]);
            auto [materialB] = material_view.get(body[1]);
            auto restitutionA = get_trimesh_restitution(*shapeA.trimesh, cp.pivotA, *cp_geom.featureA) * materialA.restitution;
            cp_mat.restitution = material_mix_restitution(restitutionA, materialB.restitution);
            return true;
        }
    } else if (mesh_shape_view.contains(body[1])) {
        auto [shapeB] = mesh_shape_view.get(body[1]);
        if (shapeB.trimesh->has_per_vertex_restitution()) {
            auto [materialA] = material_view.get(body[0]);
            auto [materialB] = material_view.get(body[1]);
            auto restitutionB = get_trimesh_restitution(*shapeB.trimesh, cp.pivotB, *cp_geom.featureB) * materialB.restitution;
            cp_mat.restitution = material_mix_restitution(materialA.restitution, restitutionB);
            return true;
        }
    } else if (paged_mesh_shape_view.contains(body[0])) {
        auto [shapeA] = paged_mesh_shape_view.get(body[0]);
        if (shapeA.trimesh->has_per_vertex_restitution()) {
            auto [materialA] = material_view.get(body[0]);
            auto [materialB] = material_view.get(body[1]);
            auto restitutionA = get_paged_mesh_restitution(shapeA, cp.pivotA, *cp_geom.featureA) * materialA.restitution;
            cp_mat.restitution = material_mix_restitution(restitutionA, materialB.restitution);
            return true;
        }
    } else if (paged_mesh_shape_view.contains(body[1])) {
        auto [shapeB] = paged_mesh_shape_view.get(body[1]);
        if (shapeB.trimesh->has_per_vertex_restitution()) {
            auto [materialA] = material_view.get(body[0]);
            auto [materialB] = material_view.get(body[1]);
            auto restitutionB = get_paged_mesh_restitution(shapeB, cp.pivotB, *cp_geom.featureB) * materialB.restitution;
            cp_mat.restitution = material_mix_restitution(materialA.restitution, restitutionB);
            return true;
        }
    }

    return false;
}

void merge_point(entt::registry &registry,
                 std::array<entt::entity, 2> body,
                 const collision_result::collision_point &rp,
                 contact_point &cp,
                 contact_point_material *cp_mat,
                 contact_point_geometry &cp_geom) {
    cp.pivotA = rp.pivotA;
    cp.pivotB = rp.pivotB;
    cp.normal = rp.normal;
    cp_geom.distance = rp.distance;
    cp_geom.normal_attachment = rp.normal_attachment;
    cp_geom.featureA = rp.featureA;
    cp_geom.featureB = rp.featureB;

    if (rp.normal_attachment != contact_normal_attachment::none) {
        auto idx = rp.normal_attachment == contact_normal_attachment::normal_on_A ? 0 : 1;
        auto &orn = registry.get<orientation>(body[idx]);
        cp_geom.local_normal = rotate(conjugate(orn), rp.normal);
    } else {
        cp_geom.local_normal = vector3_zero;
    }

    if (cp_mat) {
        try_assign_per_vertex_friction(registry, body, cp, *cp_mat, cp_geom);
        try_assign_per_vertex_restitution(registry, body, cp, *cp_mat, cp_geom);
    }
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

static void assign_material_properties(entt::registry &registry,
                                       contact_manifold &manifold,
                                       const contact_point &cp,
                                       contact_point_material &cp_mat,
                                       const contact_point_geometry &cp_geom) {
    auto material_view = registry.view<material>();
    auto [materialA] = material_view.get(manifold.body[0]);
    auto [materialB] = material_view.get(manifold.body[1]);

    auto &material_table = registry.ctx().get<material_mix_table>();

    if (auto *material = material_table.try_get({materialA.id, materialB.id})) {
        cp_mat.restitution = material->restitution;
        cp_mat.friction = material->friction;
        cp_mat.roll_friction = material->roll_friction;
        cp_mat.spin_friction = material->spin_friction;
        cp_mat.stiffness = material->stiffness;
        cp_mat.damping = material->damping;
    } else {
        if (!try_assign_per_vertex_friction(registry, manifold.body, cp, cp_mat, cp_geom)) {
            cp_mat.friction = material_mix_friction(materialA.friction, materialB.friction);
        }

        if (!try_assign_per_vertex_restitution(registry, manifold.body, cp, cp_mat, cp_geom)) {
            cp_mat.restitution = material_mix_restitution(materialA.restitution, materialB.restitution);
        }

        cp_mat.roll_friction = material_mix_roll_friction(materialA.roll_friction, materialB.roll_friction);
        cp_mat.spin_friction = material_mix_spin_friction(materialA.spin_friction, materialB.spin_friction);

        if (materialA.stiffness < large_scalar || materialB.stiffness < large_scalar) {
            cp_mat.stiffness = material_mix_stiffness(materialA.stiffness, materialB.stiffness);
            cp_mat.damping = material_mix_damping(materialA.damping, materialB.damping);
        }
    }
}

entt::entity create_contact_point(entt::registry &registry,
                                  entt::entity manifold_entity,
                                  contact_manifold &manifold,
                                  contact_manifold_state &manifold_state,
                                  const collision_result::collision_point& rp,
                                  const std::optional<transient> &transient_contact) {
    EDYN_ASSERT(length_sqr(rp.normal) > EDYN_EPSILON);
    EDYN_ASSERT(manifold_state.num_points <= max_contacts);

    auto cp = contact_point{};
    cp.pivotA = rp.pivotA;
    cp.pivotB = rp.pivotB;
    cp.normal = rp.normal;

    auto cp_geom = contact_point_geometry{};
    cp_geom.normal_attachment = rp.normal_attachment;
    cp_geom.distance = rp.distance;
    cp_geom.featureA = rp.featureA;
    cp_geom.featureB = rp.featureB;

    if (rp.normal_attachment != contact_normal_attachment::none) {
        auto idx = rp.normal_attachment == contact_normal_attachment::normal_on_A ? 0 : 1;
        auto &orn = registry.get<orientation>(manifold.body[idx]);
        cp_geom.local_normal = rotate(conjugate(orn), rp.normal);
    } else {
        cp_geom.local_normal = vector3_zero;
    }

    auto contact_entity = registry.create();

    auto cp_list = contact_point_list{};
    cp_list.parent = manifold_entity;
    cp_list.next = manifold_state.contact_entity;

    manifold_state.contact_entity = contact_entity;
    ++manifold_state.num_points;
    registry.patch<contact_manifold_state>(manifold_entity);

    // Assign material properties to contact point.
    if (registry.all_of<material>(manifold.body[0]) && registry.all_of<material>(manifold.body[1])) {
        auto cp_mat = contact_point_material{};
        assign_material_properties(registry, manifold, cp, cp_mat, cp_geom);
        registry.emplace<contact_point_material>(contact_entity, std::move(cp_mat));

        if (cp_mat.spin_friction > 0) {
            registry.emplace<contact_point_spin_friction_impulse>(contact_entity);
        }

        if (cp_mat.roll_friction > 0) {
            registry.emplace<contact_point_roll_friction_impulse>(contact_entity);
        }

        registry.emplace<contact_point_impulse>(contact_entity);
        bool needs_extras = cp_mat.stiffness < large_scalar || cp_mat.damping < large_scalar ||
                            cp_mat.spin_friction > 0 || cp_mat.roll_friction > 0;

        if (needs_extras) {
            make_constraint<contact_extras_constraint>(registry, contact_entity, manifold.body[0], manifold.body[1]);
        } else {
            make_constraint<contact_constraint>(registry, contact_entity, manifold.body[0], manifold.body[1]);
        }
    } else {
        // Create a null constraint to ensure an edge will exist in the
        // entity graph for this contact point.
        make_constraint<null_constraint>(registry, contact_entity, manifold.body[0], manifold.body[1]);
    }

    registry.emplace<contact_point_geometry>(contact_entity, std::move(cp_geom));
    registry.emplace<contact_point_list>(contact_entity, std::move(cp_list));
    registry.emplace<contact_point>(contact_entity, std::move(cp));

    if (transient_contact) {
        registry.emplace<transient>(contact_entity, *transient_contact);
    }

    return contact_entity;
}

bool should_remove_point(const contact_point &cp,
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

    return normal_dist > threshold || tangential_dist_sqr > threshold_sqr;
}

void destroy_contact_point(entt::registry &registry, entt::entity contact_entity) {
    auto &cp_list = registry.get<contact_point_list>(contact_entity);
    auto manifold_entity = cp_list.parent;
    auto &manifold_state = registry.get<contact_manifold_state>(manifold_entity);

    if (contact_entity == manifold_state.contact_entity) {
        manifold_state.contact_entity = registry.get<contact_point_list>(contact_entity).next;
    } else {
        auto current_entity = manifold_state.contact_entity;
        auto *curr_cp = &registry.get<contact_point_list>(current_entity);

        while (curr_cp->next != contact_entity) {
            current_entity = curr_cp->next;
            curr_cp = &registry.get<contact_point_list>(current_entity);
        }

        curr_cp->next = cp_list.next;
        registry.patch<contact_point_list>(current_entity);
    }

    --manifold_state.num_points;
    registry.patch<contact_manifold_state>(manifold_entity);
    registry.destroy(contact_entity);
}

void detect_collision(entt::registry &registry, std::array<entt::entity, 2> body, collision_result &result) {
    auto body_view = registry.view<AABB, shape_index, position, orientation>();
    auto &aabbA = body_view.get<AABB>(body[0]);
    auto &aabbB = body_view.get<AABB>(body[1]);
    const auto offset = vector3_one * -contact_breaking_threshold;

    // Only proceed to closest points calculation if AABBs intersect, since
    // a manifold is allowed to exist whilst the AABB separation is smaller
    // than `manifold.separation_threshold` which is greater than the
    // contact breaking threshold.
    if (intersect(aabbA.inset(offset), aabbB)) {
        auto origin_view = registry.view<origin>();
        auto &ornA = body_view.get<orientation>(body[0]);
        auto &ornB = body_view.get<orientation>(body[1]);

        auto originA = origin_view.contains(body[0]) ?
        static_cast<vector3>(origin_view.get<origin>(body[0])) :
        static_cast<vector3>(body_view.get<position>(body[0]));
        auto originB = origin_view.contains(body[1]) ?
        static_cast<vector3>(origin_view.get<origin>(body[1])) :
        static_cast<vector3>(body_view.get<position>(body[1]));

        auto shapes_views_tuple = get_tuple_of_shape_views(registry);
        auto shape_indexA = body_view.get<shape_index>(body[0]);
        auto shape_indexB = body_view.get<shape_index>(body[1]);
        auto ctx = collision_context{originA, ornA, aabbA, originB, ornB, aabbB, collision_threshold};

        visit_shape(shape_indexA, body[0], shapes_views_tuple, [&](auto &&shA) {
            visit_shape(shape_indexB, body[1], shapes_views_tuple, [&](auto &&shB) {
                collide(shA, shB, ctx, result);
            });
        });
    } else {
        result.num_points = 0;
    }
}

}
