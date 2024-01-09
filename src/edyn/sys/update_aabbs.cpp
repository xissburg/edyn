#include "edyn/sys/update_aabbs.hpp"
#include "edyn/comp/origin.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/comp/position.hpp"
#include "edyn/comp/aabb.hpp"
#include "edyn/comp/tag.hpp"
#include "edyn/comp/island.hpp"
#include "edyn/util/aabb_util.hpp"
#include "edyn/util/island_util.hpp"
#include <entt/entity/registry.hpp>

namespace edyn {

template<typename ShapeType>
AABB updated_aabb(const ShapeType &shape, const vector3 &pos, const quaternion &orn) {
    return shape_aabb(shape, pos, orn);
}

template<>
AABB updated_aabb(const polyhedron_shape &polyhedron,
                  const vector3 &pos, const quaternion &orn) {
    // `shape_aabb(const polyhedron_shape &, ...)` rotates each vertex of a
    // polyhedron to calculate the AABB. Specialize `updated_aabb` for
    // polyhedrons to use the rotated mesh.
    auto aabb = point_cloud_aabb(polyhedron.rotated->vertices);
    aabb.min += pos;
    aabb.max += pos;
    return aabb;
}

template<typename ShapeType, typename TransformView, typename OriginView>
void update_aabb(entt::entity entity, ShapeType &shape, TransformView &tr_view,
                 OriginView &origin_view) {
    auto [orn, aabb] = tr_view.template get<orientation, AABB>(entity);
    auto origin = origin_view.contains(entity) ?
        static_cast<vector3>(origin_view.template get<edyn::origin>(entity)) :
        static_cast<vector3>(tr_view.template get<edyn::position>(entity));
    aabb = updated_aabb(shape, origin, orn);
}

void update_aabb(entt::registry &registry, entt::entity entity) {
    auto origin_view = registry.view<origin>();
    auto tr_view = registry.view<position, orientation, AABB>();

    visit_shape(registry, entity, [&](auto &&shape) {
        update_aabb(entity, shape, tr_view, origin_view);
    });
}

template<typename ShapeType>
void update_aabbs(entt::registry &registry) {
    auto tr_view = registry.view<position, orientation, ShapeType, AABB, dynamic_tag>(exclude_sleeping_disabled);
    auto origin_view = registry.view<origin>();

    for (auto entity : tr_view) {
        auto &shape = tr_view.template get<ShapeType>(entity);
        update_aabb(entity, shape, tr_view, origin_view);
    }

    // TODO: only update AABB of kinematic entities that have moved.
    auto kin_tr_view = registry.view<position, orientation, ShapeType, AABB, kinematic_tag>();
    for (auto entity : kin_tr_view) {
        auto &shape = kin_tr_view.template get<ShapeType>(entity);
        update_aabb(entity, shape, kin_tr_view, origin_view);
    }
}

template<typename... Ts>
void update_aabbs(entt::registry &registry, std::tuple<Ts...>) {
    (update_aabbs<Ts>(registry), ...);
}

void update_aabbs(entt::registry &registry) {
    // Update AABBs for all shapes that can be transformed.
    update_aabbs(registry, dynamic_shapes_tuple);
}

void update_island_aabbs(entt::registry &registry) {
    auto aabb_view = registry.view<AABB>();
    auto procedural_view = registry.view<procedural_tag>();

    registry.view<island, island_AABB>(exclude_sleeping_disabled)
        .each([&](island &island, island_AABB &aabb) {
        auto is_first_node = true;

        for (auto entity : island.nodes) {
            if (!procedural_view.contains(entity) || !aabb_view.contains(entity)) {
                continue;
            }

            auto &node_aabb = aabb_view.get<AABB>(entity);

            if (is_first_node) {
                aabb = {node_aabb};
                is_first_node = false;
            } else {
                aabb = {enclosing_aabb(aabb, node_aabb)};
            }
        }
    });
}

}
