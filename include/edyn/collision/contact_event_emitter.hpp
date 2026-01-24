#ifndef EDYN_COLLISION_CONTACT_EVENT_EMITTER_HPP
#define EDYN_COLLISION_CONTACT_EVENT_EMITTER_HPP

#include <entt/entity/fwd.hpp>
#include <entt/signal/sigh.hpp>
#include "edyn/collision/contact_manifold.hpp"

namespace edyn {

class contact_event_emitter {
public:
    contact_event_emitter(entt::registry &);

    // Cannot be copied because `entt::scoped_connection` deletes its copy ctor.
    contact_event_emitter(const contact_event_emitter &) = delete;
    contact_event_emitter(contact_event_emitter &&) = default;

    auto contact_started_sink() {
        return entt::sink{m_contact_started_signal};
    }

    auto contact_ended_sink() {
        return entt::sink{m_contact_ended_signal};
    }

    void on_construct_contact_manifold(entt::registry &, entt::entity);
    void on_update_contact_manifold(entt::registry &, entt::entity);
    void on_destroy_contact_manifold(entt::registry &, entt::entity);

private:
    entt::sigh<void(entt::entity)> m_contact_started_signal;
    entt::sigh<void(entt::entity)> m_contact_ended_signal;
    std::vector<entt::scoped_connection> m_connections;
};

}

#endif // EDYN_COLLISION_CONTACT_EVENT_EMITTER_HPP
