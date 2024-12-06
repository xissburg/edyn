#include "edyn/collision/raycast.hpp"
#include "edyn/collision/tree_node.hpp"
#include "edyn/comp/island.hpp"
#include "edyn/math/geom.hpp"
#include "edyn/math/math.hpp"
#include "edyn/math/transform.hpp"
#include "edyn/simulation/stepper_async.hpp"
#include "edyn/math/triangle.hpp"
#include <unordered_set>

namespace edyn {

raycast_id_type raycast_async(entt::registry &registry, vector3 p0, vector3 p1,
                              const raycast_delegate_type &delegate,
                              const std::vector<entt::entity> &ignore_entities) {
    auto &stepper = registry.ctx().get<stepper_async>();
    return stepper.raycast(p0, p1, delegate, ignore_entities);
}

raycast_result raycast(entt::registry &registry, vector3 p0, vector3 p1,
                       const std::vector<entt::entity> &ignore_entities) {
    auto index_view = registry.view<shape_index>();
    auto tr_view = registry.view<position, orientation>();
    auto origin_view = registry.view<origin>();
    auto shape_views_tuple = get_tuple_of_shape_views(registry);

    entt::entity hit_entity {entt::null};
    shape_raycast_result result;

    auto raycast_shape = [&](entt::entity entity) {
        auto sh_idx = index_view.get<shape_index>(entity);
        auto pos = origin_view.contains(entity) ?
            static_cast<vector3>(origin_view.get<origin>(entity)) :
            tr_view.get<position>(entity);
        auto orn = tr_view.get<orientation>(entity);
        auto ctx = raycast_context{pos, orn, p0, p1};

        visit_shape(sh_idx, entity, shape_views_tuple, [&](auto &&shape) {
            auto res = shape_raycast(shape, ctx);

            if (res.fraction < result.fraction) {
                result = res;
                hit_entity = entity;
            }
        });
    };

    auto &bphase = registry.ctx().get<broadphase>();
    bphase.raycast(p0, p1, [&](entt::entity entity) {
        if (!vector_contains(ignore_entities, entity)) {
            raycast_shape(entity);
        }
    });

    return {result, hit_entity};
}

shape_raycast_result shape_raycast(const box_shape &box, const raycast_context &ctx) {
    // Reference: Real-Time Collision Detection - Christer Ericson,
    // Section 5.3.3 - Intersecting Ray or Segment Against Box.
    auto p0 = to_object_space(ctx.p0, ctx.pos, ctx.orn);
    auto p1 = to_object_space(ctx.p1, ctx.pos, ctx.orn);
    auto dir = p1 - p0;

    auto t_min = -EDYN_SCALAR_MAX;
    auto t_max = EDYN_SCALAR_MAX;
    auto face_idx = SIZE_MAX;

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

    auto result = shape_raycast_result{};
    result.fraction = t_min;
    result.normal = box.get_face_normal(face_idx, ctx.orn);
    result.info_var = box_raycast_info{face_idx};

    return result;
}

shape_raycast_result shape_raycast(const cylinder_shape &cylinder, const raycast_context &ctx) {
    scalar fraction_in, fraction_out;
    auto intersect_result = intersect_ray_cylinder(ctx.p0, ctx.p1, ctx.pos, ctx.orn,
                                                   cylinder.radius, cylinder.half_length,
                                                   cylinder.axis, fraction_in, fraction_out);

    if (intersect_result.kind == intersect_ray_cylinder_result::kind::distance_greater_than_radius) {
        return {};
    }

    auto cyl_vertices = cylinder.get_vertices(ctx.pos, ctx.orn);
    auto ray_dir = ctx.p1 - ctx.p0;
    auto cyl_dir = cyl_vertices[1] - cyl_vertices[0];
    auto cyl_dir_norm = normalize(cyl_dir);
    auto face_idx = dot(ctx.p0 - ctx.pos, cyl_dir) < 0 ? size_t(0) : size_t(1);
    auto face_normal = cyl_dir_norm * (face_idx == 0 ? -1 : 1);
    auto radius = cylinder.radius;
    auto radius_sqr = square(radius);

    if (intersect_result.kind == intersect_ray_cylinder_result::kind::parallel_directions) {
        // Cylinder and segment are parallel. Check if segment intersects a
        // cap face.
        vector3 closest; scalar fraction;
        auto dist_sqr = closest_point_line(ctx.p0, ray_dir, cyl_vertices[face_idx], fraction, closest);

        if (dist_sqr > radius_sqr) {
            return {};
        }

        auto result = shape_raycast_result{};
        result.fraction = fraction;
        result.normal = face_normal;
        result.info_var = cylinder_raycast_info{cylinder_feature::face, face_idx};

        return result;
    }

    // Line intersects infinite cylinder. Check if it's within finite cylinder.
    auto intersection = lerp(ctx.p0, ctx.p1, fraction_in);
    scalar projections[] = {
        dot(intersection - cyl_vertices[0], cyl_dir_norm),
        dot(intersection - cyl_vertices[1], cyl_dir_norm)
    };

    if (projections[0] > 0 && projections[1] < 0) {
        // Intersection lies on the side of cylinder.
        auto result = shape_raycast_result{};
        result.fraction = fraction_in;
        result.normal = intersect_result.normal / std::sqrt(intersect_result.dist_sqr);
        result.info_var = cylinder_raycast_info{cylinder_feature::side_edge};
        return result;
    }

    // Intersection point is beyond cylinder with finite length.
    // Intersect line with plane parallel to cap face.
    auto t = dot(cyl_vertices[face_idx] - ctx.p0, face_normal) / dot(ray_dir, face_normal);
    intersection = lerp(ctx.p0, ctx.p1, t);
    auto dist_sqr = distance_sqr(intersection, cyl_vertices[face_idx]);

    if (dist_sqr > radius_sqr) {
        return {};
    }

    auto result = shape_raycast_result{};
    result.fraction = t;
    result.normal = face_normal;
    result.info_var = cylinder_raycast_info{cylinder_feature::face, face_idx};

    return result;
}

shape_raycast_result shape_raycast(const sphere_shape &sphere, const raycast_context &ctx) {
    scalar t;

    if (!intersect_ray_sphere(ctx.p0, ctx.p1, ctx.pos, sphere.radius, t)) {
        return {};
    }

    auto intersection = lerp(ctx.p0, ctx.p1, t);

    shape_raycast_result result;
    result.fraction = t;
    result.normal = normalize(intersection - ctx.pos);
    return result;
}

shape_raycast_result shape_raycast(const capsule_shape &capsule, const raycast_context &ctx) {
    scalar u_in, u_out;
    auto intersect_result = intersect_ray_cylinder(ctx.p0, ctx.p1, ctx.pos, ctx.orn,
                                                   capsule.radius, capsule.half_length, capsule.axis, u_in, u_out);

    if (intersect_result.kind == intersect_ray_cylinder_result::kind::distance_greater_than_radius) {
        return {};
    }

    auto vertices = capsule.get_vertices(ctx.pos, ctx.orn);
    auto cap_dir = vertices[1] - vertices[0];
    auto radius = capsule.radius;

    auto intersect_hemisphere = [&](size_t hemi_idx) -> shape_raycast_result {
        scalar fraction;

        if (!intersect_ray_sphere(ctx.p0, ctx.p1, vertices[hemi_idx], radius, fraction)) {
            return {};
        }

        auto intersection = lerp(ctx.p0, ctx.p1, u_in);
        auto normal = normalize(intersection - ctx.pos);

        auto result = shape_raycast_result{};
        result.fraction = fraction;
        result.normal = normal;
        result.info_var = capsule_raycast_info{capsule_feature::hemisphere, hemi_idx};
        return result;
    };

    if (intersect_result.kind == intersect_ray_cylinder_result::kind::parallel_directions) {
        // Capsule and segment are parallel. Check if segment intersects a
        // hemisphere.
        auto hemi_idx = dot(ctx.p0 - ctx.pos, cap_dir) < 0 ? 0 : 1;
        return intersect_hemisphere(hemi_idx);
    }

    // Line intersects infinite cylinder. Check if it's within finite
    // cylindrical section.
    auto intersection = lerp(ctx.p0, ctx.p1, u_in);

    scalar projections[] = {
        dot(intersection - vertices[0], cap_dir),
        dot(intersection - vertices[1], cap_dir)
    };

    if (projections[0] > 0 && projections[1] < 0) {
        // Intersection lies on the side of cylindrical section.
        auto result = shape_raycast_result{};
        result.fraction = u_in;
        result.normal = intersect_result.normal / std::sqrt(intersect_result.dist_sqr);
        result.info_var = capsule_raycast_info{capsule_feature::side};

        return result;
    }

    // Intersection point is beyond cylindrical section.
    // Intersect line with hemisphere.
    auto hemi_idx = projections[0] < 0 ? 0 : 1;
    return intersect_hemisphere(hemi_idx);
}

shape_raycast_result shape_raycast(const polyhedron_shape &poly, const raycast_context &ctx) {
    // Reference: Real-Time Collision Detection - Christer Ericson,
    // Section 5.3.8 - Intersecting Ray or Segment Against Convex Polyhedron.
    auto p0 = to_object_space(ctx.p0, ctx.pos, ctx.orn);
    auto p1 = to_object_space(ctx.p1, ctx.pos, ctx.orn);
    auto d = p1 - p0;
    auto t0 = -EDYN_SCALAR_MAX;
    auto t1 = EDYN_SCALAR_MAX;
    auto intersect_face_idx = SIZE_MAX;
    const auto &mesh = *poly.mesh;

    for (size_t face_idx = 0; face_idx < mesh.num_faces(); ++face_idx) {
        auto vertex = mesh.vertices[mesh.first_vertex_index(face_idx)];
        auto normal = mesh.normals[face_idx];
        auto dist = dot(vertex - p0, normal);
        auto denom = dot(normal, d);

        // Test if segment runs parallel to the plane.
        if (std::abs(denom) < EDYN_EPSILON) {
            // Segment does not intersect polyhedron if there's any face that is
            // parallel to it and it lies in front of the face.
            if (dist > 0) {
                return {};
            }
        } else {
            // Compute parametrized `t` value for intersection with plane of
            // current face.
            auto t = dist / denom;

            if (denom < 0) {
                // When entering face, assign `t` to `t0` if `t` is greater.
                if (t > t0) {
                    t0 = t;
                    intersect_face_idx = face_idx;
                }
            } else {
                // When exiting face, assign `t` to `t1` if `t` is smaller.
                if (t < t1) {
                    t1 = t;
                }
            }

            // Intersection range has become empty.
            if (t0 > t1) {
                return {};
            }
        }
    }

    // Check if intersection range intersects the [0, 1] interval.
    if ((t0 < 0 && t1 < 0) || (t0 > 1 && t1 > 1)) {
        return {};
    }

    EDYN_ASSERT(intersect_face_idx != SIZE_MAX);

    auto result = shape_raycast_result{};
    result.fraction = clamp_unit(t0);
    result.normal = rotate(ctx.orn, mesh.normals[intersect_face_idx]);
    result.info_var = polyhedron_raycast_info{intersect_face_idx};

    return result;
}

shape_raycast_result shape_raycast(const compound_shape &compound, const raycast_context &ctx) {
    auto p0 = to_object_space(ctx.p0, ctx.pos, ctx.orn);
    auto p1 = to_object_space(ctx.p1, ctx.pos, ctx.orn);
    shape_raycast_result result;

    compound.raycast(p0, p1, [&](auto &&shape, auto node_index) {
        auto &node = compound.nodes[node_index];
        auto child_ctx = raycast_context{};
        child_ctx.p0 = p0;
        child_ctx.p1 = p1;
        child_ctx.pos = node.position;
        child_ctx.orn = node.orientation;
        auto child_result = shape_raycast(shape, child_ctx);

        if (child_result.fraction < result.fraction) {
            result.fraction = child_result.fraction;
            result.normal = rotate(ctx.orn, child_result.normal);
            auto info = compound_raycast_info{node_index};
            // Obtain and assign relevant child info.
            using child_info_var_t = decltype(info.child_info_var);
            std::visit([&](auto &&child_info) {
                using child_info_t = std::decay_t<decltype(child_info)>;
                if constexpr(variant_has_type<child_info_t, child_info_var_t>::value) {
                    info.child_info_var = child_info;
                }
            }, child_result.info_var);
            result.info_var = info;
        }
    });

    return result;
}

shape_raycast_result shape_raycast(const plane_shape &plane, const raycast_context &ctx) {
    auto c = plane.normal * plane.constant;
    auto d = dot(ctx.p1 - ctx.p0, plane.normal);
    auto e = dot(c - ctx.p0, plane.normal);

    if (std::abs(d) < EDYN_EPSILON) {
        // Ray is parallel to plane.
        if (std::abs(e) < EDYN_EPSILON) {
            auto result = shape_raycast_result{};
            result.fraction = 0;
            result.normal = plane.normal;
            return result;
        } else {
            return {};
        }
    } else {
        auto t = e / d;
        auto result = shape_raycast_result{};
        result.fraction = t;
        result.normal = plane.normal;
        return result;
    }
}

shape_raycast_result shape_raycast(const mesh_shape &mesh, const raycast_context &ctx) {
    auto &trimesh = mesh.trimesh;
    shape_raycast_result result;

    trimesh->raycast(ctx.p0, ctx.p1, [&](auto tri_idx) {
        auto vertices = trimesh->get_triangle_vertices(tri_idx);
        auto normal = trimesh->get_triangle_normal(tri_idx);
        auto t = scalar(0);

        if (!intersect_segment_triangle(ctx.p0, ctx.p1, vertices, normal, t)) {
            return;
        }

        if (t < result.fraction) {
            result.fraction = t;
            result.normal = normal;
            result.info_var = mesh_raycast_info{tri_idx};
        }
    });

    return result;
}

shape_raycast_result shape_raycast(const paged_mesh_shape &paged_mesh, const raycast_context &ctx) {
    shape_raycast_result result;

    paged_mesh.trimesh->raycast_cached(ctx.p0, ctx.p1, [&](auto submesh_idx, auto tri_idx) {
        auto trimesh = paged_mesh.trimesh->get_submesh(submesh_idx);
        auto vertices = trimesh->get_triangle_vertices(tri_idx);
        auto normal = trimesh->get_triangle_normal(tri_idx);
        auto t = scalar(0);

        if (!intersect_segment_triangle(ctx.p0, ctx.p1, vertices, normal, t)) {
            return;
        }

        // Intersection is inside triangle.
        if (t < result.fraction) {
            result.fraction = t;
            result.normal = normal;
            result.info_var = paged_mesh_raycast_info{submesh_idx, tri_idx};
        }
    });

    return result;
}

}
