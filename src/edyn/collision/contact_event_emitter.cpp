#include "edyn/collision/contact_event_emitter.hpp"
#include "edyn/collision/contact_manifold_events.hpp"

namespace edyn {

contact_event_emitter::contact_event_emitter(entt::registry &registry)
    : m_registry(&registry)
{
    registry.on_destroy<contact_manifold>().connect<&contact_event_emitter::on_destroy_contact_manifold>(*this);
}

contact_event_emitter::~contact_event_emitter() {
    m_registry->on_destroy<contact_manifold>().disconnect<&contact_event_emitter::on_destroy_contact_manifold>(*this);
}

void contact_event_emitter::consume_events() {
    for (auto [entity, events] : m_registry->view<contact_manifold_events>().each()) {
        // Contact could have ended and started again in the same step. Do not
        // generate event in that case.
        if (events.contact_started && !events.contact_ended) {
            m_contact_started_signal.publish(entity);
        }

        for (unsigned i = 0; i < events.num_contacts_created; ++i) {
            m_contact_point_created_signal.publish(entity, events.contacts_created[i]);
        }

        for (unsigned i = 0; i < events.num_contacts_destroyed; ++i) {
            m_contact_point_destroyed_signal.publish(entity, events.contacts_destroyed[i]);
        }

        if (events.contact_ended && !events.contact_started) {
            m_contact_ended_signal.publish(entity);
        }

        events = {};
    }
}

void contact_event_emitter::on_destroy_contact_manifold(entt::registry &registry, entt::entity entity) {
    // Trigger contact destroyed events.
    auto &manifold = registry.get<contact_manifold>(entity);

    if (manifold.num_points > 0) {
        for (unsigned i = 0; i < manifold.num_points; ++i) {
            m_contact_point_destroyed_signal.publish(entity, manifold.ids[i]);
        }

        m_contact_ended_signal.publish(entity);
    }
}

}
