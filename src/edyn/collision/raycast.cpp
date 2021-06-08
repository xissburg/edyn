#include "edyn/collision/raycast.hpp"
#include "edyn/collision/tree_node.hpp"
#include "edyn/comp/position.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/comp/shape_index.hpp"
#include "edyn/collision/tree_view.hpp"
#include "edyn/collision/broadphase_main.hpp"
#include "edyn/collision/broadphase_worker.hpp"
#include "edyn/math/geom.hpp"
#include "edyn/math/quaternion.hpp"
#include "edyn/math/scalar.hpp"
#include "edyn/math/vector3.hpp"
#include "edyn/shapes/box_shape.hpp"
#include "edyn/math/math.hpp"
#include "edyn/config/constants.hpp"
#include "edyn/shapes/cylinder_shape.hpp"
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
    // Let a plane be defined by the ray and the vector orthogonal to the
    // cylinder axis and the ray (i.e. their cross product). This plane cuts
    // the cylinder and their intersection is an ellipse with vertical half
    // length equals to the cylinder radius and horizontal half length bigger
    // than that. First, the parameters for the closest point between the lines
    // spanned by the cylinder axis and the ray are found. By subtracting an
    // amount from the parameter for the ray, the intersection point can be
    // found.
    auto cyl_vertices = cylinder.get_vertices(ctx.pos, ctx.orn);
    auto ray_dir = ctx.p1 - ctx.p0;
    auto cyl_dir = cyl_vertices[1] - cyl_vertices[0];
    auto cyl_dir_norm = normalize(cyl_dir);
    auto face_idx = dot(ctx.p0 - ctx.pos, cyl_dir) < 0 ? 0 : 1;
    auto face_normal = cyl_dir_norm * (face_idx == 0 ? -1 : 1);
    auto radius_sqr = square(cylinder.radius);
    auto tolerance_sqr = square(raycast_feature_tolerance);
    scalar s, t;

    if (!closest_point_line_line(cyl_vertices[0], cyl_vertices[1], ctx.p0, ctx.p1, s, t)) {
        // Cylinder and segment are parallel. Check if segment intersects a
        // cap face.
        vector3 closest; scalar u;
        auto dist_sqr = closest_point_line(ctx.p0, ray_dir, cyl_vertices[face_idx], u, closest);

        if (dist_sqr > radius_sqr) {
            return {};
        }

        auto result = shape_raycast_result{};
        result.proportion = u;
        result.normal = face_normal;
        result.feature.index = face_idx;

        if (dist_sqr > radius_sqr - tolerance_sqr) {
            result.feature.feature = cylinder_feature::cap_edge;
        } else {
            result.feature.feature = cylinder_feature::face;
        }

        return result;
    }

    auto closest_cyl = lerp(cyl_vertices[0], cyl_vertices[1], s);
    auto closest_ray = lerp(ctx.p0, ctx.p1, t);
    auto normal = closest_ray - closest_cyl;
    auto dist_sqr = length_sqr(normal);

    if (dist_sqr > radius_sqr) {
        return {};
    }

    // Offset `t` backwards to place it where the intersection happens.
    auto d = ctx.p1 - ctx.p0;
    auto e = cyl_vertices[1] - cyl_vertices[0];
    auto dd = dot(d, d);
    auto ee = dot(e, e);
    auto de = dot(d, e);
    auto g_sqr = (radius_sqr - dist_sqr) * ee / (dd * ee - de * de);
    auto u = t - std::sqrt(g_sqr);
    auto intersection = lerp(ctx.p0, ctx.p1, u);
    scalar projections[] = {
        dot(intersection - cyl_vertices[0], cyl_dir_norm),
        dot(intersection - cyl_vertices[1], cyl_dir_norm)
    };

    if (projections[0] > 0 && projections[1] < 0) {
        // Intersection lies on the side of cylinder.
        auto result = shape_raycast_result{};
        result.proportion = u;
        result.normal = normal / std::sqrt(dist_sqr);

        if (projections[0] < raycast_feature_tolerance) {
            result.feature.feature = cylinder_feature::cap_edge;
            result.feature.index = 0;
        } else if (projections[1] > -raycast_feature_tolerance) {
            result.feature.feature = cylinder_feature::cap_edge;
            result.feature.index = 1;
        } else {
            result.feature.feature = cylinder_feature::side_edge;
        }
        return result;
    }

    // Intersect line with plane parallel to cap face.
    t = dot(cyl_vertices[face_idx] - ctx.p0, face_normal) / dot(ray_dir, face_normal);
    intersection = lerp(ctx.p0, ctx.p1, t);
    dist_sqr = distance_sqr(intersection, cyl_vertices[face_idx]);

    if (dist_sqr > radius_sqr) {
        return {};
    }

    auto result = shape_raycast_result{};
    result.proportion = t;
    result.normal = face_normal;
    result.feature.feature = cylinder_feature::face;
    result.feature.index = face_idx;

    if (dist_sqr > radius_sqr - tolerance_sqr) {
        result.feature.feature = cylinder_feature::cap_edge;
    } else {
        result.feature.feature = cylinder_feature::face;
    }

    return result;
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
