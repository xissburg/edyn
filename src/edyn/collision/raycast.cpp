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
#include "edyn/shapes/polyhedron_shape.hpp"
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


struct intersect_ray_cylinder_result {
    enum class kind {
        parallel_directions,
        distance_greater_than_radius,
        intersects
    };

    kind kind;
    scalar dist_sqr;
    vector3 normal;
};

intersect_ray_cylinder_result intersect_ray_cylinder(vector3 p0, vector3 p1, vector3 pos, quaternion orn, scalar radius, scalar half_length, scalar &u) {
    // Let a plane be defined by the ray and the vector orthogonal to the
    // cylinder axis and the ray (i.e. their cross product). This plane cuts
    // the cylinder and their intersection is an ellipse with vertical half
    // length equals to the cylinder radius and horizontal half length bigger
    // than that. First, the parameters for the closest point between the lines
    // spanned by the cylinder axis and the ray are found. By subtracting an
    // amount from the parameter for the ray, the intersection point can be
    // found.
    auto cyl_dir = quaternion_x(orn);
    vector3 cyl_vertices[] = {
        pos + cyl_dir * half_length,
        pos - cyl_dir * half_length
    };
    scalar s, t;

    if (!closest_point_line_line(cyl_vertices[0], cyl_vertices[1], p0, p1, s, t)) {
        return {intersect_ray_cylinder_result::kind::parallel_directions};
    }

    auto radius_sqr = square(radius);
    auto closest_cyl = lerp(cyl_vertices[0], cyl_vertices[1], s);
    auto closest_ray = lerp(p0, p1, t);
    auto normal = closest_ray - closest_cyl;
    auto dist_sqr = length_sqr(normal);

    // Distance between lines bigger than radius.
    if (dist_sqr > radius_sqr) {
        return {intersect_ray_cylinder_result::kind::distance_greater_than_radius};
    }

    // Offset `t` backwards to place it where the intersection happens.
    auto d = p1 - p0;
    auto e = cyl_vertices[1] - cyl_vertices[0];
    auto dd = dot(d, d);
    auto ee = dot(e, e);
    auto de = dot(d, e);
    auto g_sqr = (radius_sqr - dist_sqr) * ee / (dd * ee - de * de);
    auto g = std::sqrt(g_sqr);
    u = t - g;
    return {intersect_ray_cylinder_result::kind::intersects, dist_sqr, normal};
}

shape_raycast_result raycast(const cylinder_shape &cylinder, const raycast_context &ctx) {
    scalar u;
    auto intersect_result = intersect_ray_cylinder(ctx.p0, ctx.p1, ctx.pos, ctx.orn,
                                                   cylinder.radius, cylinder.half_length, u);

    if (intersect_result.kind == intersect_ray_cylinder_result::kind::distance_greater_than_radius) {
        return {};
    }

    auto cyl_vertices = cylinder.get_vertices(ctx.pos, ctx.orn);
    auto ray_dir = ctx.p1 - ctx.p0;
    auto cyl_dir = cyl_vertices[1] - cyl_vertices[0];
    auto cyl_dir_norm = normalize(cyl_dir);
    auto face_idx = dot(ctx.p0 - ctx.pos, cyl_dir) < 0 ? 0 : 1;
    auto face_normal = cyl_dir_norm * (face_idx == 0 ? -1 : 1);
    auto radius = cylinder.radius;
    auto radius_sqr = square(radius);
    auto tolerance = raycast_feature_tolerance;

    if (intersect_result.kind == intersect_ray_cylinder_result::kind::parallel_directions) {
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

        if (dist_sqr > square(radius - tolerance)) {
            result.feature.feature = cylinder_feature::cap_edge;
        } else {
            result.feature.feature = cylinder_feature::face;
        }

        return result;
    }

    // Line intersects infinite cylinder. Check if it's within finite cylinder.
    auto intersection = lerp(ctx.p0, ctx.p1, u);
    scalar projections[] = {
        dot(intersection - cyl_vertices[0], cyl_dir_norm),
        dot(intersection - cyl_vertices[1], cyl_dir_norm)
    };

    if (projections[0] > 0 && projections[1] < 0) {
        // Intersection lies on the side of cylinder.
        auto result = shape_raycast_result{};
        result.proportion = u;
        result.normal = intersect_result.normal / std::sqrt(intersect_result.dist_sqr);

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

    // Intersection point is beyond cylinder with finite length.
    // Intersect line with plane parallel to cap face.
    auto t = dot(cyl_vertices[face_idx] - ctx.p0, face_normal) / dot(ray_dir, face_normal);
    intersection = lerp(ctx.p0, ctx.p1, t);
    auto dist_sqr = distance_sqr(intersection, cyl_vertices[face_idx]);

    if (dist_sqr > radius_sqr) {
        return {};
    }

    auto result = shape_raycast_result{};
    result.proportion = t;
    result.normal = face_normal;
    result.feature.index = face_idx;

    if (dist_sqr > square(radius - tolerance)) {
        result.feature.feature = cylinder_feature::cap_edge;
    } else {
        result.feature.feature = cylinder_feature::face;
    }

    return result;
}

bool intersect_ray_sphere(vector3 p0, vector3 p1, vector3 pos, scalar radius, scalar &t) {
    // Reference: Real-Time Collision Detection - Christer Ericson,
    // Section 5.3.2 - Intersecting Ray or Segment Against Sphere.
    // Substitute parametrized line function into sphere equation and
    // solve quadratic.
    auto d = p1 - p0;
    auto m = p0 - pos;
    auto a = dot(d, d);
    auto b = dot(m, d);
    auto c = dot(m, m) - radius * radius;

    // Exit if p0 is outside sphere and ray is pointing away from sphere.
    if (c > 0 && b > 0) {
        return false;
    }

    auto discr = b * b - a * c;

    // A negative discriminant corresponds to ray missing sphere.
    if (discr < 0) {
        return false;
    }

    // Ray intersects sphere. Compute smallest t.
    t = (-b - std::sqrt(discr)) / a;
    // If t is negative, ray started inside sphere so clamp it to zero.
    t = std::max(scalar(0), t);

    return true;
}

shape_raycast_result raycast(const sphere_shape &sphere, const raycast_context &ctx) {
    scalar t;

    if (!intersect_ray_sphere(ctx.p0, ctx.p1, ctx.pos, sphere.radius, t)) {
        return {};
    }

    auto intersection = lerp(ctx.p0, ctx.p1, t);

    shape_raycast_result result;
    result.proportion = t;
    result.normal = normalize(intersection - ctx.pos);
    return result;
}

shape_raycast_result raycast(const capsule_shape &capsule, const raycast_context &ctx) {
    scalar u;
    auto intersect_result = intersect_ray_cylinder(ctx.p0, ctx.p1, ctx.pos, ctx.orn,
                                                   capsule.radius, capsule.half_length, u);

    if (intersect_result.kind == intersect_ray_cylinder_result::kind::distance_greater_than_radius) {
        return {};
    }

    auto vertices = capsule.get_vertices(ctx.pos, ctx.orn);
    auto cap_dir = vertices[1] - vertices[0];
    auto radius = capsule.radius;

    auto intersect_hemisphere = [&] (int hemi_idx) -> shape_raycast_result {
        if (!intersect_ray_sphere(ctx.p0, ctx.p1, vertices[hemi_idx], radius, u)) {
            return {};
        }

        auto intersection = lerp(ctx.p0, ctx.p1, u);
        auto normal = normalize(intersection - ctx.pos);

        auto result = shape_raycast_result{};
        result.proportion = u;
        result.normal = normal;
        result.feature.feature = capsule_feature::hemisphere;
        result.feature.index = hemi_idx;
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
    auto intersection = lerp(ctx.p0, ctx.p1, u);

    scalar projections[] = {
        dot(intersection - vertices[0], cap_dir),
        dot(intersection - vertices[1], cap_dir)
    };

    if (projections[0] > 0 && projections[1] < 0) {
        // Intersection lies on the side of cylindrical section.
        auto result = shape_raycast_result{};
        result.proportion = u;
        result.normal = intersect_result.normal / std::sqrt(intersect_result.dist_sqr);
        result.feature.feature = capsule_feature::side;

        return result;
    }

    // Intersection point is beyond cylindrical section.
    // Intersect line with hemisphere.
    auto hemi_idx = projections[0] < 0 ? 0 : 1;
    return intersect_hemisphere(hemi_idx);
}

shape_raycast_result raycast(const polyhedron_shape &poly, const raycast_context &ctx) {
    // Reference: Real-Time Collision Detection - Christer Ericson,
    // Section 5.3.8 - Intersecting Ray or Segment Against Convex Polyhedron.
    auto p0 = to_object_space(ctx.p0, ctx.pos, ctx.orn);
    auto p1 = to_object_space(ctx.p1, ctx.pos, ctx.orn);
    auto d = p1 - p0;
    auto t0 = scalar(0);
    auto t1 = scalar(1);
    auto intersect_face_idx = SIZE_MAX;

    for (size_t face_idx = 0; face_idx < poly.mesh->num_faces(); ++face_idx) {
        auto vertex = poly.mesh->vertices[poly.mesh->first_vertex_index(face_idx)];
        auto normal = poly.mesh->normals[face_idx];
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

    auto intersection = lerp(p0, p1, t0);
    auto tolerance = raycast_feature_tolerance;
    auto tolerance_sqr = tolerance * tolerance;
    auto result = shape_raycast_result{};
    result.proportion = t0;
    result.normal = rotate(ctx.orn, poly.mesh->normals[intersect_face_idx]);
    result.feature.feature = polyhedron_feature::face;
    result.feature.index = intersect_face_idx;

    for (size_t i = 0; i < poly.mesh->vertex_count(intersect_face_idx); ++i) {
        auto v_idx = poly.mesh->face_vertex_index(intersect_face_idx, i);
        auto vertex = poly.mesh->vertices[v_idx];

        if (distance_sqr(vertex, intersection) < tolerance_sqr) {
            result.feature.feature = polyhedron_feature::vertex;
            result.feature.index = v_idx;
        }
    }

    return result;
}

shape_raycast_result raycast(const compound_shape &compound, const raycast_context &ctx) {
    auto p0 = to_object_space(ctx.p0, ctx.pos, ctx.orn);
    auto p1 = to_object_space(ctx.p1, ctx.pos, ctx.orn);
    shape_raycast_result result;

    compound.raycast(p0, p1, [&] (auto &&shape, auto node_index) {
        auto &node = compound.nodes[node_index];
        auto child_ctx = raycast_context{};
        child_ctx.p0 = p0;
        child_ctx.p1 = p1;
        child_ctx.pos = node.position;
        child_ctx.orn = node.orientation;
        auto child_result = raycast(shape, child_ctx);

        if (child_result.proportion < result.proportion) {
            result = child_result;
            result.normal = rotate(ctx.orn, child_result.normal);
            result.feature.index = node_index;
        }
    });

    return result;
}

shape_raycast_result raycast(const plane_shape &plane, const raycast_context &ctx) {
    auto c = plane.normal * plane.constant;
    auto d = dot(ctx.p1 - ctx.p0, plane.normal);
    auto e = dot(c - ctx.p0, plane.normal);

    if (std::abs(d) < EDYN_EPSILON) {
        // Ray is parallel to plane.
        if (std::abs(e) < EDYN_EPSILON) {
            auto result = shape_raycast_result{};
            result.proportion = 0;
            result.normal = plane.normal;
            return result;
        } else {
            return {};
        }
    } else {
        auto t = e / d;
        auto result = shape_raycast_result{};
        result.proportion = t;
        result.normal = plane.normal;
        return result;
    }
}

shape_raycast_result raycast(const mesh_shape &mesh, const raycast_context &ctx) {
    shape_raycast_result result;

    mesh.trimesh->raycast(ctx.p0, ctx.p1, [&] (auto tri_idx) {
        auto vertices = mesh.trimesh->get_triangle_vertices(tri_idx);
        auto normal = mesh.trimesh->get_triangle_normal(tri_idx);
        auto d = dot(ctx.p1 - ctx.p0, normal);
        auto e = dot(vertices[0] - ctx.p0, normal);
        auto t = scalar(0);

        if (std::abs(d) > EDYN_EPSILON) {
            t = e / d;
        } else {
            // Ray is parallel to plane. Do not continue if ray is not
            // contained in plane.
            if (std::abs(e) > EDYN_EPSILON) {
                return;
            }
        }

        auto intersection = lerp(ctx.p0, ctx.p1, t);

        for (size_t i = 0; i < 3; ++i) {
            auto v0 = vertices[i];
            auto v1 = vertices[(i + 1) % 3];
            auto edge_dir = v1 - v0;
            auto tangent = cross(normal, edge_dir);

            if (dot(tangent, intersection - v0) < 0) {
                return;
            }
        }

        // Intersection is inside triangle.
        if (t < result.proportion) {
            result.proportion = t;
            result.normal = normal;
            result.feature.index = tri_idx;
        }
    });

    return result;
}

shape_raycast_result raycast(const paged_mesh_shape &, const raycast_context &ctx) {
    return {};
}

}
