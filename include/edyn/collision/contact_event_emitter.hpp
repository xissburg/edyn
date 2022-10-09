#ifndef EDYN_COLLISION_CONTACT_EVENT_EMITTER_HPP
#define EDYN_COLLISION_CONTACT_EVENT_EMITTER_HPP

#include <entt/entity/fwd.hpp>
#include <entt/signal/sigh.hpp>
#include "edyn/collision/contact_manifold.hpp"

namespace edyn {

class contact_event_emitter {
public:
    contact_event_emitter(entt::registry &);
    ~contact_event_emitter();

    void consume_events();

    auto contact_started_sink() {
        return entt::sink{m_contact_started_signal};
    }

    auto contact_ended_sink() {
        return entt::sink{m_contact_ended_signal};
    }

    auto contact_point_created_sink() {
        return entt::sink{m_contact_point_created_signal};
    }

    auto contact_point_destroyed_sink() {
        return entt::sink{m_contact_point_destroyed_signal};
    }

    void on_destroy_contact_manifold(entt::registry &, entt::entity);

private:
    entt::registry *m_registry;
    entt::sigh<void(entt::entity)> m_contact_started_signal;
    entt::sigh<void(entt::entity)> m_contact_ended_signal;
    entt::sigh<void(entt::entity, contact_manifold::contact_id_type)> m_contact_point_created_signal;
    entt::sigh<void(entt::entity, contact_manifold::contact_id_type)> m_contact_point_destroyed_signal;
};

}

#endif // EDYN_COLLISION_CONTACT_EVENT_EMITTER_HPP
