#include "edyn/sys/update_inertias.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/comp/inertia.hpp"
#include "edyn/comp/tag.hpp"
#include "edyn/util/island_util.hpp"
#include <entt/entity/fwd.hpp>
#include <entt/entity/registry.hpp>
#include <vector>

namespace edyn {

template<typename OrnInertiaView>
void update_inertia(entt::entity entity, OrnInertiaView &view) {
    auto [orn, inv_I, inv_IW] = view.template get<orientation, inertia_inv, inertia_world_inv>(entity);
    auto basis = to_matrix3x3(orn);
    inv_IW = basis * inv_I * transpose(basis);
}

void update_inertias(entt::registry &registry) {
    auto view = registry.view<orientation, inertia_inv, inertia_world_inv, dynamic_tag>(exclude_sleeping_disabled);
    for (auto entity : view) {
        update_inertia(entity, view);
    }
}

void update_inertia(entt::registry &registry, entt::entity entity) {
    auto view = registry.view<orientation, inertia_inv, inertia_world_inv, dynamic_tag>();
    update_inertia(entity, view);
}

template<typename It>
void update_inertias(entt::registry &registry, It first, It last) {
    auto view = registry.view<orientation, inertia_inv, inertia_world_inv, dynamic_tag>();

    for (; first != last; ++first) {
        auto entity = *first;

        if (view.contains(entity)) {
            update_inertia(entity, view);
        }
    }
}

void update_inertias(entt::registry &registry, const entt::sparse_set &entities) {
    update_inertias(registry, entities.begin(), entities.end());
}

void update_inertias(entt::registry &registry, const std::vector<entt::entity> &entities) {
    update_inertias(registry, entities.begin(), entities.end());
}

}
