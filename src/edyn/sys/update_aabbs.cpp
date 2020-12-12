#include "edyn/sys/update_aabbs.hpp"
#include "edyn/util/island_util.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/comp/position.hpp"
#include "edyn/comp/shape.hpp"
#include "edyn/comp/aabb.hpp"
#include <entt/entt.hpp>

namespace edyn {

void update_aabbs(entt::registry &registry) {
    auto view = registry.view<position, orientation, shape, AABB>(entt::exclude<disabled_tag>);
    view.each([] (auto, auto &pos, auto &orn, auto &sh, auto &aabb) {
        std::visit([&] (auto &&s) {
            aabb = s.aabb(pos, orn);
        }, sh.var);
    });
}

}