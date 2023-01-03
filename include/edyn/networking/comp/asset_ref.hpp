#ifndef EDYN_NETWORKING_COMP_ASSET_REF_HPP
#define EDYN_NETWORKING_COMP_ASSET_REF_HPP

#include <entt/entity/fwd.hpp>

namespace edyn {

/**
 * @brief Reference to an asset that contains information about a remote entity
 * to be instantiated and replicated locally. Sent by the server to clients
 * when a new entity enters their AABB of interest thus allowing the client to
 * obtain the asset and instantiate the entity with the respective internal id.
 */
struct asset_ref {
    // Asset id.
    entt::id_type asset_id;
    // Id of entity inside of asset. The components in the asset for this id
    // must be assigned to the entity that owns this component.
    entt::id_type internal_id;
};

struct asset_factory {
    using create_func_t = void(entt::registry &, entt::entity);
    create_func_t *create;
};

template<typename Archive>
void serialize(Archive &archive, asset_ref &ref) {
    archive(ref.asset_id);
    archive(ref.internal_id);
}

}

#endif // EDYN_NETWORKING_COMP_ASSET_REF_HPP
