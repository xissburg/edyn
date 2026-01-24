#include "edyn/collision/contact_event_emitter.hpp"
#include "edyn/collision/contact_manifold.hpp"

using namespace entt::literals;

namespace edyn {

constexpr auto storage_name = "manifold_state_prev"_hs;

contact_event_emitter::contact_event_emitter(entt::registry &registry) {
    m_connections.push_back(registry.on_construct<contact_manifold_state>().connect<&contact_event_emitter::on_construct_contact_manifold>(*this));
    m_connections.push_back(registry.on_update<contact_manifold_state>().connect<&contact_event_emitter::on_update_contact_manifold>(*this));
    m_connections.push_back(registry.on_destroy<contact_manifold_state>().connect<&contact_event_emitter::on_destroy_contact_manifold>(*this));
}

void contact_event_emitter::on_construct_contact_manifold(entt::registry &registry, entt::entity entity) {
    auto &manifold_state = registry.get<const contact_manifold_state>(entity);
    registry.storage<contact_manifold_state>(storage_name).emplace(entity, manifold_state);
}

void contact_event_emitter::on_update_contact_manifold(entt::registry &registry, entt::entity entity) {
    auto &manifold_state = registry.get<const contact_manifold_state>(entity);
    auto &old = registry.storage<contact_manifold_state>(storage_name).get(entity);

    if (manifold_state.contact_entity != entt::null && old.contact_entity == entt::null) {
        m_contact_started_signal.publish(entity);
    } else if (manifold_state.contact_entity == entt::null && old.contact_entity != entt::null) {
        m_contact_ended_signal.publish(entity);
    }

    old = manifold_state;
}

void contact_event_emitter::on_destroy_contact_manifold(entt::registry &registry, entt::entity entity) {
    registry.storage(storage_name)->erase(entity);
    auto &manifold_state = registry.get<const contact_manifold_state>(entity);

    if (manifold_state.contact_entity != entt::null) {
        m_contact_ended_signal.publish(entity);
    }
}

}
