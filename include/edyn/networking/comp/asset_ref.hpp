#ifndef EDYN_NETWORKING_COMP_ASSET_REF_HPP
#define EDYN_NETWORKING_COMP_ASSET_REF_HPP

#include <cstdint>
#include <entt/entity/fwd.hpp>
#include <map>

namespace edyn {

/**
 * @brief Reference to an asset that contains information about a remote entity
 * to be instantiated and replicated locally. Sent by the server to clients
 * when a new entity enters their AABB of interest thus allowing the client to
 * obtain the asset and instantiate the entity with the respective internal id.
 */
struct asset_ref {
    // Asset id.
    entt::id_type id;
    uint32_t version;
    // Maps internal asset ids to instantiated entities.
    std::map<entt::id_type, entt::entity> entity_map;
};

template<typename Archive>
void serialize(Archive &archive, asset_ref &ref) {
    archive(ref.id, ref.version);
    archive(ref.entity_map);
}

/**
 * @brief Assigned to assets after they're instantiated as a replica of a
 * remote asset, thus having the local entities linked to the remote counterparts.
 */
struct asset_linked_tag {};

}

#endif // EDYN_NETWORKING_COMP_ASSET_REF_HPP
