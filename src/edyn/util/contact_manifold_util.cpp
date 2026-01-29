#include "edyn/util/contact_manifold_util.hpp"
#include "edyn/collision/contact_manifold.hpp"
#include "edyn/collision/contact_manifold_map.hpp"
#include <entt/entity/registry.hpp>

namespace edyn {

bool manifold_exists(entt::registry &registry, entt::entity first, entt::entity second) {
    return manifold_exists(registry, entity_pair{first, second});
}

bool manifold_exists(entt::registry &registry, entity_pair entities) {
    auto &manifold_map = registry.ctx().get<contact_manifold_map>();
    return manifold_map.contains(entities);
}

entt::entity get_manifold_entity(const entt::registry &registry, entt::entity first, entt::entity second) {
    return get_manifold_entity(registry, entity_pair{first, second});
}

entt::entity get_manifold_entity(const entt::registry &registry, entity_pair entities) {
    auto &manifold_map = registry.ctx().get<contact_manifold_map>();
    return manifold_map.get(entities);
}

void clear_contact_manifold(entt::registry &registry, entt::entity manifold_entity) {
    auto &manifold_state = registry.get<contact_manifold_state>(manifold_entity);
    auto contact_entity = manifold_state.contact_entity;
    manifold_state = {};
    registry.patch<contact_manifold_state>(manifold_entity);

    auto cp_view = registry.view<contact_point_list>();

    while (contact_entity != entt::null) {
        auto next = std::get<0>(cp_view.get(contact_entity)).next;
        registry.destroy(contact_entity);
        contact_entity = next;
    }
}

}
