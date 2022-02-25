#ifndef EDYN_NETWORKING_ENTITY_OWNER_HPP
#define EDYN_NETWORKING_ENTITY_OWNER_HPP

#include <entt/entity/fwd.hpp>
#include <entt/entity/entity.hpp>

namespace edyn {

/**
 * @brief Component assigned to entities that are owned by a remote client.
 */
struct entity_owner {
    entt::entity client_entity {entt::null};
};

template<typename Archive>
void serialize(Archive &archive, entity_owner &owner) {
    archive(owner.client_entity);
}

}

#endif // EDYN_NETWORKING_ENTITY_OWNER_HPP
