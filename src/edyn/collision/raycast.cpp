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
    auto p0 = to_object_space(ctx.p0, ctx.pos, ctx.orn);
    auto p1 = to_object_space(ctx.p1, ctx.pos, ctx.orn);
    auto dir = p1 - p0;

    auto t_min = scalar(0);
    auto t_max = EDYN_SCALAR_MAX;
    auto face_idx = size_t{};

    for (auto i = 0; i < 3; ++i) {
        if (std::abs(dir[i]) < EDYN_EPSILON) {
            // Ray is parallel to face.
            if (std::abs(p0[i]) > box.half_extents[i]) {
                return {};
            }
        } else {
            auto d_inv = scalar(1) / dir[i];
            auto t1 = (-box.half_extents[i] - p0[i]) * d_inv;
            auto t2 = (+box.half_extents[i] - p0[i]) * d_inv;

            if (t1 > t2) {
                std::swap(t1, t2);
            }

            if (t1 > t_min) {
                face_idx = i * 2 + (p0[i] > 0 ? 0 : 1);
            }

            t_min = std::max(t_min, t1);
            t_max = std::min(t_max, t2);

            if (t_min > t_max) {
                return {};
            }
        }
    }

    auto result = shape_raycast_result{};
    result.proportion = t_min;
    result.normal = box.get_face_normal(face_idx, ctx.orn);

    auto intersection = lerp(p0, p1, t_min);
    auto trimmed_half_extents = box.half_extents - vector3_one * raycast_feature_tolerance;

    if (face_idx == 0 || face_idx == 1) { // X face
        auto positive_face = intersection.x > 0;

        if (intersection.y > trimmed_half_extents.y) {
            if (intersection.z > trimmed_half_extents.z) {
                result.feature.feature = box_feature::vertex;
                result.feature.index = positive_face ? 0 : 4;
            } else if (intersection.z < -trimmed_half_extents.z) {
                result.feature.feature = box_feature::vertex;
                result.feature.index = positive_face ? 3 : 5;
            } else {
                result.feature.feature = box_feature::edge;
                result.feature.index = positive_face ? 3 : 4;
            }
        } else if (intersection.y < -trimmed_half_extents.y) {
            if (intersection.z > trimmed_half_extents.z) {
                result.feature.feature = box_feature::vertex;
                result.feature.index = positive_face ? 1 : 7;
            } else if (intersection.z < -trimmed_half_extents.z) {
                result.feature.feature = box_feature::vertex;
                result.feature.index = positive_face ? 2 : 6;
            } else {
                result.feature.feature = box_feature::edge;
                result.feature.index = positive_face ? 1 : 6;
            }
        } else if (intersection.z > trimmed_half_extents.z) {
            result.feature.feature = box_feature::edge;
            result.feature.index = positive_face ? 0 : 7;
        } else if (intersection.z < -trimmed_half_extents.z) {
            result.feature.feature = box_feature::edge;
            result.feature.index = positive_face ? 2 : 5;
        } else {
            result.feature.feature = box_feature::face;
            result.feature.index = face_idx;
        }
    }
    else if (face_idx == 2 || face_idx == 3) { // Y face
        auto positive_face = intersection.y > 0;

        if (intersection.x > trimmed_half_extents.x) {
            if (intersection.z > trimmed_half_extents.z) {
                result.feature.feature = box_feature::vertex;
                result.feature.index = positive_face ? 0 : 1;
            } else if (intersection.z < -trimmed_half_extents.z) {
                result.feature.feature = box_feature::vertex;
                result.feature.index = positive_face ? 3 : 2;
            } else {
                result.feature.feature = box_feature::edge;
                result.feature.index = positive_face ? 3 : 1;
            }
        } else if (intersection.x < -trimmed_half_extents.x) {
            if (intersection.z > trimmed_half_extents.z) {
                result.feature.feature = box_feature::vertex;
                result.feature.index = positive_face ? 4 : 7;
            } else if (intersection.z < -trimmed_half_extents.z) {
                result.feature.feature = box_feature::vertex;
                result.feature.index = positive_face ? 5 : 6;
            } else {
                result.feature.feature = box_feature::edge;
                result.feature.index = positive_face ? 4 : 6;
            }
        } else if (intersection.z > trimmed_half_extents.z) {
            result.feature.feature = box_feature::edge;
            result.feature.index = positive_face ? 8 : 9;
        } else if (intersection.z < -trimmed_half_extents.z) {
            result.feature.feature = box_feature::edge;
            result.feature.index = positive_face ? 11 : 10;
        } else {
            result.feature.feature = box_feature::face;
            result.feature.index = face_idx;
        }
    }
    else if (face_idx == 4 || face_idx == 5) { // Z face
        auto positive_face = intersection.z > 0;

        if (intersection.x > trimmed_half_extents.x) {
            if (intersection.y > trimmed_half_extents.y) {
                result.feature.feature = box_feature::vertex;
                result.feature.index = positive_face ? 0 : 3;
            } else if (intersection.y < -trimmed_half_extents.y) {
                result.feature.feature = box_feature::vertex;
                result.feature.index = positive_face ? 1 : 2;
            } else {
                result.feature.feature = box_feature::edge;
                result.feature.index = positive_face ? 0 : 2;
            }
        } else if (intersection.x < -trimmed_half_extents.x) {
            if (intersection.y > trimmed_half_extents.y) {
                result.feature.feature = box_feature::vertex;
                result.feature.index = positive_face ? 4 : 5;
            } else if (intersection.y < -trimmed_half_extents.y) {
                result.feature.feature = box_feature::vertex;
                result.feature.index = positive_face ? 7 : 6;
            } else {
                result.feature.feature = box_feature::edge;
                result.feature.index = positive_face ? 7 : 5;
            }
        } else if (intersection.y > trimmed_half_extents.y) {
            result.feature.feature = box_feature::edge;
            result.feature.index = positive_face ? 8 : 11;
        } else if (intersection.y < -trimmed_half_extents.y) {
            result.feature.feature = box_feature::edge;
            result.feature.index = positive_face ? 9 : 10;
        } else {
            result.feature.feature = box_feature::face;
            result.feature.index = face_idx;
        }
    }

    return result;
}

shape_raycast_result raycast(const cylinder_shape &, const raycast_context &ctx) {
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
