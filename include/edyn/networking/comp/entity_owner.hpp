#ifndef EDYN_NETWORKING_ENTITY_OWNER_HPP
#define EDYN_NETWORKING_ENTITY_OWNER_HPP

#include <entt/entity/fwd.hpp>

namespace edyn {

struct entity_owner {
    entt::entity client_entity;
};

template<typename Archive>
void serialize(Archive &archive, entity_owner &owner) {
    archive(owner.client_entity);
}

}

#endif // EDYN_NETWORKING_ENTITY_OWNER_HPP
