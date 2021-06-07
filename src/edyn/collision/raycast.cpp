#include "edyn/collision/raycast.hpp"
#include "edyn/collision/tree_node.hpp"
#include "edyn/comp/position.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/comp/shape_index.hpp"
#include "edyn/collision/tree_view.hpp"
#include "edyn/collision/broadphase_main.hpp"
#include "edyn/collision/broadphase_worker.hpp"
#include "edyn/math/quaternion.hpp"
#include "edyn/shapes/box_shape.hpp"
#include "edyn/math/math.hpp"
#include "edyn/config/constants.hpp"
#include "edyn/shapes/shapes.hpp"
#include <entt/entt.hpp>

namespace edyn {

raycast_result raycast(entt::registry &registry, vector3 p0, vector3 p1) {
    auto index_view = registry.view<shape_index>();
    auto tr_view = registry.view<position, orientation>();
    auto tree_view_view = registry.view<tree_view>();
    auto shape_views_tuple = get_tuple_of_shape_views(registry);

    entt::entity hit_entity {entt::null};
    shape_raycast_result result;

    auto raycast_shape = [&] (entt::entity entity) {
        auto sh_idx = index_view.get(entity);
        auto pos = tr_view.get<position>(entity);
        auto orn = tr_view.get<orientation>(entity);
        auto ctx = raycast_context{pos, orn, p0, p1};

        visit_shape(sh_idx, entity, shape_views_tuple, [&] (auto &&shape) {
            auto res = raycast(shape, ctx);

            if (res.proportion < result.proportion) {
                result = res;
                hit_entity = entity;
            }
        });
    };

    if (registry.try_ctx<broadphase_main>() != nullptr) {
        auto &bphase = registry.ctx<broadphase_main>();
        bphase.raycast_islands(p0, p1, [&] (entt::entity island_entity) {
            auto &tree_view = tree_view_view.get(island_entity);
            tree_view.raycast(p0, p1, [&] (tree_node_id_t id) {
                auto entity = tree_view.get_node(id).entity;
                raycast_shape(entity);
            });
        });

        bphase.raycast_non_procedural(p0, p1, raycast_shape);
    } else {
        auto &bphase = registry.ctx<broadphase_worker>();
        bphase.raycast(p0, p1, raycast_shape);
    }

    return {result, hit_entity};
}

shape_raycast_result raycast(const box_shape &box, const raycast_context &ctx) {
    // Reference: Real-Time Collision Detection - Christer Ericson,
    // Section 5.3.3 - Intersecting Ray or Segment Against Box.
    auto p0 = to_object_space(ctx.p0, ctx.pos, ctx.orn);
    auto p1 = to_object_space(ctx.p1, ctx.pos, ctx.orn);
    auto dir = p1 - p0;

    auto t_min = scalar(0);
    auto t_max = EDYN_SCALAR_MAX;
    auto face_idx = size_t{};

    for (auto i = 0; i < 3; ++i) {
        if (std::abs(dir[i]) < EDYN_EPSILON) {
            // Ray is parallel to face. If any coordinate value is beyond the
            // extents there's no intersection.
            if (std::abs(p0[i]) > box.half_extents[i]) {
                return {};
            }
        } else {
            // Find parameter for ray where it intersects both parallel faces.
            auto d_inv = scalar(1) / dir[i];
            auto t1 = (-box.half_extents[i] - p0[i]) * d_inv;
            auto t2 = (+box.half_extents[i] - p0[i]) * d_inv;

            // Make t1 the intersection with the closest face.
            if (t1 > t2) {
                std::swap(t1, t2);
            }

            // Select this face index if it is further.
            if (t1 > t_min) {
                face_idx = i * 2 + (p0[i] > 0 ? 0 : 1);
            }

            // Clamp parameters: maximize t_min and minimize t_max.
            t_min = std::max(t_min, t1);
            t_max = std::min(t_max, t2);

            // Intersection is empty.
            if (t_min > t_max) {
                return {};
            }
        }
    }

    // Find feature closest to point of intersection.
    auto intersection = lerp(p0, p1, t_min);
    auto [feature, feature_idx] =
        box.get_closest_feature_on_face(face_idx, intersection, raycast_feature_tolerance);

    auto result = shape_raycast_result{};
    result.proportion = t_min;
    result.normal = box.get_face_normal(face_idx, ctx.orn);
    result.feature.feature = feature;
    result.feature.index = feature_idx;

    return result;
}

shape_raycast_result raycast(const cylinder_shape &cylinder, const raycast_context &ctx) {
    return {};
}

shape_raycast_result raycast(const sphere_shape &, const raycast_context &ctx) {
    return {};
}

shape_raycast_result raycast(const capsule_shape &, const raycast_context &ctx) {
    return {};
}

shape_raycast_result raycast(const polyhedron_shape &, const raycast_context &ctx) {
    return {};
}

shape_raycast_result raycast(const compound_shape &, const raycast_context &ctx) {
    return {};
}

shape_raycast_result raycast(const plane_shape &, const raycast_context &ctx) {
    return {};
}

shape_raycast_result raycast(const mesh_shape &, const raycast_context &ctx) {
    return {};
}

shape_raycast_result raycast(const paged_mesh_shape &, const raycast_context &ctx) {
    return {};
}

}
