#include "edyn/sys/update_aabbs.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/comp/position.hpp"
#include "edyn/comp/shape.hpp"
#include "edyn/comp/aabb.hpp"
#include "edyn/comp/tag.hpp"
#include "edyn/util/aabb_util.hpp"
#include <entt/entity/registry.hpp>

namespace edyn {

template<typename S>
AABB updated_aabb(const S &sh, const vector3 &pos, const quaternion &orn) {
    return shape_aabb(sh, pos, orn);
}

template<>
AABB updated_aabb(const polyhedron_shape &sh, const vector3 &pos, const quaternion &orn) {
    // Polyhedron AABB is calculated in `update_rotated_meshes` along with
    // the vertex rotations.
    return {};
}

void update_aabbs(entt::registry &registry) {
    auto view = registry.view<position, orientation, shape, AABB>();
    view.each([] (position &pos, orientation &orn, shape &sh, AABB &aabb) {
        std::visit([&] (auto &&s) {
            aabb = updated_aabb(s, pos, orn);
        }, sh.var);
    });
}

}