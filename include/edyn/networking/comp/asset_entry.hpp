#ifndef EDYN_NETWORKING_COMP_ASSET_ENTRY_HPP
#define EDYN_NETWORKING_COMP_ASSET_ENTRY_HPP

#include "edyn/networking/util/component_index_type.hpp"
#include <entt/core/fwd.hpp>
#include <entt/entity/fwd.hpp>
#include <vector>

namespace edyn {

struct asset_entry {
    entt::entity asset_entity;
    entt::id_type id;
    std::vector<component_index_type> sync_indices;
};

}

#endif // EDYN_NETWORKING_COMP_ASSET_ENTRY_HPP
