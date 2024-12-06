#include "edyn/collision/contact_signal.hpp"
#include "edyn/collision/contact_event_emitter.hpp"

namespace edyn {

entt::sink<entt::sigh<void(entt::entity)>> on_contact_started(entt::registry &registry) {
    return registry.ctx().get<contact_event_emitter>().contact_started_sink();
}

entt::sink<entt::sigh<void(entt::entity)>> on_contact_ended(entt::registry &registry) {
    return registry.ctx().get<contact_event_emitter>().contact_ended_sink();
}

entt::sink<entt::sigh<void(entt::entity, contact_manifold::contact_id_type)>> on_contact_point_created(entt::registry &registry) {
    return registry.ctx().get<contact_event_emitter>().contact_point_created_sink();
}

entt::sink<entt::sigh<void(entt::entity, contact_manifold::contact_id_type)>> on_contact_point_destroyed(entt::registry &registry) {
    return registry.ctx().get<contact_event_emitter>().contact_point_destroyed_sink();
}

}
