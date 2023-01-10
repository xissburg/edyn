#ifndef EDYN_NETWORKING_COMP_ASSET_ENTRY_HPP
#define EDYN_NETWORKING_COMP_ASSET_ENTRY_HPP

#include "edyn/networking/util/component_index_type.hpp"
#include <entt/core/fwd.hpp>
#include <entt/entity/fwd.hpp>
#include <vector>

namespace edyn {

/**
 * @brief An entry contained in an asset.
 * @remark When an asset is replicated, the components referenced in `sync_indices`
 * will be included in a registry snapshot and be sent to the other end to ensure
 * the corresponding entity will be instantiated with the correct initial state.
 */
struct asset_entry {
    // Asset it belongs to.
    entt::entity asset_entity;
    // Unique id inside of asset.
    entt::id_type id;
    // Components to be synchronized when replicated.
    std::vector<component_index_type> sync_indices;
};

}

#endif // EDYN_NETWORKING_COMP_ASSET_ENTRY_HPP
