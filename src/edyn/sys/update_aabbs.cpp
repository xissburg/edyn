#include "edyn/sys/update_aabbs.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/comp/position.hpp"
#include "edyn/comp/shape.hpp"
#include "edyn/comp/aabb.hpp"
#include "edyn/comp/tag.hpp"
#include <entt/entity/registry.hpp>

namespace edyn {

void update_aabbs(entt::registry &registry) {
    auto view = registry.view<position, orientation, shape, AABB>();
    view.each([] (position &pos, orientation &orn, shape &sh, AABB &aabb) {
        std::visit([&] (auto &&s) {
            aabb = s.aabb(pos, orn);
        }, sh.var);
    });
}

}