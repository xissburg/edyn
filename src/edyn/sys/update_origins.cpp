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

void update_origin(const position &pos, const orientation &orn, const center_of_mass &com, origin &orig) {
    orig = to_world_space(-com, pos, orn);
}

void update_origins(entt::registry &registry) {
    registry.view<const position, const orientation, const center_of_mass, origin>(exclude_sleeping_disabled).each(update_origin);
}

template<typename It>
void update_origins(entt::registry &registry, It first, It last) {
    auto view = registry.view<const position, const orientation, const center_of_mass, origin>();

    for (; first != last; ++first) {
        auto entity = *first;

        if (view.contains(entity)) {
            auto [pos, orn, com, orig] = view.get(entity);
            update_origin(pos, orn, com, orig);
        }
    }
}

void update_origins(entt::registry &registry, const entt::sparse_set &entities) {
    update_origins(registry, entities.begin(), entities.end());
}

void update_origins(entt::registry &registry, const std::vector<entt::entity> &entities) {
    update_origins(registry, entities.begin(), entities.end());
}

}
