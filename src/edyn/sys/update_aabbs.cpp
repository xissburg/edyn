#include "edyn/sys/update_aabbs.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/comp/position.hpp"
#include "edyn/comp/shape.hpp"
#include "edyn/comp/aabb.hpp"
#include "edyn/comp/tag.hpp"
#include "edyn/util/aabb_util.hpp"
#include <entt/entity/registry.hpp>

namespace edyn {

static AABB updated_aabb(const plane_shape &sh, const vector3 &pos, const quaternion &orn) {
    return shape_aabb(sh, pos, orn);
}

static AABB updated_aabb(const sphere_shape &sh, const vector3 &pos, const quaternion &orn) {
    return shape_aabb(sh, pos, orn);
}

static AABB updated_aabb(const cylinder_shape &sh, const vector3 &pos, const quaternion &orn) {
    return shape_aabb(sh, pos, orn);
}

static AABB updated_aabb(const capsule_shape &sh, const vector3 &pos, const quaternion &orn) {
    return shape_aabb(sh, pos, orn);
}

static AABB updated_aabb(const box_shape &sh, const vector3 &pos, const quaternion &orn) {
    return shape_aabb(sh, pos, orn);
}

static AABB updated_aabb(const polyhedron_shape &sh, const vector3 &pos, const quaternion &orn) {
    // Polyhedron AABB is calculated in `update_rotated_meshes` while along with
    // the vertex rotations.
    return {};
}

static AABB updated_aabb(const mesh_shape &sh, const vector3 &pos, const quaternion &orn) {
    return shape_aabb(sh, pos, orn);
}

static AABB updated_aabb(const paged_mesh_shape &sh, const vector3 &pos, const quaternion &orn) {
    return shape_aabb(sh, pos, orn);
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