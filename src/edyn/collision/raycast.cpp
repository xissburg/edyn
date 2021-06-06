#include "edyn/collision/raycast.hpp"
#include "edyn/comp/position.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/comp/shape_index.hpp"
#include "edyn/collision/tree_view.hpp"
#include "edyn/collision/broadphase_main.hpp"
#include "edyn/collision/broadphase_worker.hpp"
#include "edyn/shapes/shapes.hpp"
#include <entt/entt.hpp>

namespace edyn {

raycast_result raycast(entt::registry &registry, vector3 p0, vector3 p1) {
    auto index_view = registry.view<shape_index>();
    auto tr_view = registry.view<position, orientation>();
    auto tree_view_view = registry.view<tree_view>();
    auto shape_views_tuple = get_tuple_of_shape_views(registry);
    raycast_result result;

    if (registry.try_ctx<broadphase_main>() != nullptr) {
        auto &bphase = registry.ctx<broadphase_main>();
        bphase.raycast_islands(p0, p1, [&] (entt::entity island_entity) {
            auto &tree_view = tree_view_view.get(island_entity);
            tree_view.raycast(p0, p1, [&] (entt::entity entity) {
                auto sh_idx = index_view.get(entity);
                auto pos = tr_view.get<position>(entity);
                auto orn = tr_view.get<orientation>(entity);
                auto ctx = raycast_context{pos, orn, p0, p1};

                visit_shape(sh_idx, entity, shape_views_tuple, [&] (auto &&shape) {
                    auto t = raycast(shape, ctx);
                    if (t < result.proportion) {
                        result.proportion = t;
                        result.entity = entity;
                    }
                });
            });
        });

        bphase.raycast_non_procedural(p0, p1, [&] (entt::entity entity) {

        });
    } else {
        auto &bphase = registry.ctx<broadphase_worker>();
        bphase.raycast(p0, p1, [&] (entt::entity entity) {
            auto sh_idx = index_view.get(entity);
            auto pos = tr_view.get<position>(entity);
            auto orn = tr_view.get<orientation>(entity);
            auto ctx = raycast_context{pos, orn, p0, p1};

            visit_shape(sh_idx, entity, shape_views_tuple, [&] (auto &&shape) {
                auto t = raycast(shape, ctx);
                if (t < result.proportion) {
                    result.proportion = t;
                    result.entity = entity;
                }
            });
        });
    }

    return result;
}

}
