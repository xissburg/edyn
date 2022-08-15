#ifndef EDYN_COLLISION_CONTACT_SIGNAL_HPP
#define EDYN_COLLISION_CONTACT_SIGNAL_HPP

#include <entt/entity/fwd.hpp>
#include <entt/signal/fwd.hpp>
#include "edyn/collision/contact_manifold.hpp"

namespace edyn {

/**
 * @brief Signal triggered when a contact starts.
 * A contact is considered to start when the first contact point is added to a
 * manifold, i.e. when the number of points in a manifold becomes greater than
 * zero.
 * @param registry Data source.
 * @return Sink to observe contact started events.
 */
entt::sink<entt::sigh<void(entt::entity)>> on_contact_started(entt::registry &);

/**
 * @brief Signal triggered when a contact ends.
 * A contact ends when the last point is destroyed in a contact manifold, i.e.
 * when the number of points goes to zero, or when a manifold is destroyed due
 * to AABB separation.
 * @param registry Data source.
 * @return Sink to observe contact ended events.
 */
entt::sink<entt::sigh<void(entt::entity)>> on_contact_ended(entt::registry &);

/**
 * @brief Signal triggered when a contact point is created.
 * This event is also triggered right after a contact started event, for each
 * point that the contact has started with.
 * The signal emits the manifold entity and the contact point id in that manifold.
 * @param registry Data source.
 * @return Sink to observe contact point creation events.
 */
entt::sink<entt::sigh<void(entt::entity, contact_manifold::contact_id_type)>>
on_contact_point_created(entt::registry &);

/**
 * @brief Signal triggered when a contact point is destroyed.
 * This event is also triggered for each contact point before a contact ended
 * event.
 * The signal emits the manifold entity and the contact point id in that manifold.
 * @param registry Data source.
 * @return Sink to observe contact point destruction events.
 */
entt::sink<entt::sigh<void(entt::entity, contact_manifold::contact_id_type)>>
on_contact_point_destroyed(entt::registry &);

}

#endif // EDYN_COLLISION_CONTACT_SIGNAL_HPP
