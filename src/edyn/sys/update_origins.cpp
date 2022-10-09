#include "edyn/sys/update_origins.hpp"
#include "edyn/comp/center_of_mass.hpp"
#include "edyn/comp/origin.hpp"
#include "edyn/comp/position.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/comp/tag.hpp"
#include "edyn/math/transform.hpp"
#include "edyn/util/island_util.hpp"
#include <entt/entity/registry.hpp>

namespace edyn {

void update_origins(entt::registry &registry) {
    registry.view<position, orientation, center_of_mass, origin>(exclude_sleeping_disabled)
        .each([](position &pos, orientation &orn, center_of_mass &com, origin &orig) {
        orig = to_world_space(-com, pos, orn);
    });
}

}
