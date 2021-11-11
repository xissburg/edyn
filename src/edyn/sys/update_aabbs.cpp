#include "edyn/sys/update_aabbs.hpp"
#include "edyn/comp/origin.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/comp/position.hpp"
#include "edyn/comp/aabb.hpp"
#include "edyn/comp/tag.hpp"
#include "edyn/util/aabb_util.hpp"
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
void update_aabb(entt::entity entity, ShapeType &shape, TransformView &tr_view, OriginView &origin_view) {
    auto [orn, aabb] = tr_view.template get<orientation, AABB>(entity);
    auto origin = origin_view.contains(entity) ?
        static_cast<vector3>(origin_view.template get<edyn::origin>(entity)) :
        static_cast<vector3>(tr_view.template get<edyn::position>(entity));
    aabb = updated_aabb(shape, origin, orn);
}

void update_aabb(entt::registry &registry, entt::entity entity) {
    auto tr_view = registry.view<position, orientation, AABB>();
    auto origin_view = registry.view<origin>();

    visit_shape(registry, entity, [&] (auto &&shape) {
        update_aabb(entity, shape, tr_view, origin_view);
    });
}

template<typename ShapeType>
void update_aabbs(entt::registry &registry) {
    auto origin_view = registry.view<origin>();
    auto tr_view = registry.view<position, orientation, ShapeType, AABB>();

    for (auto entity : tr_view) {
        auto &shape = tr_view.template get<ShapeType>(entity);
        update_aabb(entity, shape, tr_view, origin_view);
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

}
