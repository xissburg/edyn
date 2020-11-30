#ifndef EDYN_PARALLEL_MERGE_MERGE_UTIL_HPP
#define EDYN_PARALLEL_MERGE_MERGE_UTIL_HPP

#include <unordered_set>
#include <entt/entity/registry.hpp>
#include "edyn/parallel/merge/merge_component.hpp"
#include "edyn/parallel/registry_delta.hpp"
#include "edyn/util/entity_map.hpp"

namespace edyn {

template<merge_type MergeType, typename Component, typename Member>
void merge_unordered_set(const Component *old_comp, 
                         Component &new_comp, 
                         Member Component:: *member, 
                         merge_context &ctx) {

    std::unordered_set<entt::entity> entities;

    for (auto remote_entity : new_comp.*member) {
        if (!ctx.map->has_rem(remote_entity)) continue;
        auto local_entity = ctx.map->remloc(remote_entity);
        entities.insert(local_entity);
    }

    new_comp.*member = entities;

    if constexpr(MergeType == merge_type::updated) {
        // Reinsert entities from old which are still valid and haven't been
        // moved to another island due to a split.
        std::vector<std::unordered_set<entt::entity>> split;

        if (!ctx.delta->m_split_connected_components.empty()) {
            // The first set contains the components left in the source island worker,
            // so it must not be considered.
            for (auto it = std::next(ctx.delta->m_split_connected_components.begin()); it != ctx.delta->m_split_connected_components.end(); ++it) {
                std::unordered_set<entt::entity> connected;

                for (auto remote_entity : *it) {
                    if (!ctx.map->has_rem(remote_entity)) continue;
                    auto local_entity = ctx.map->remloc(remote_entity);
                    connected.insert(local_entity);
                }

                split.push_back(connected);
            }
        }

        for (auto old_entity : old_comp->*member) {
            if (!ctx.registry->valid(old_entity)) continue;

            if (new_comp.*member.count(old_entity)) continue;

            bool moved = false;
            // Ignore entity if it has moved to another island due to a split.
            for (auto &connected : split) {
                if (connected.count(old_entity)) {
                    moved = true;
                    break;
                }
            }

            if (moved) continue;

            new_comp.*member.insert(old_entity);
        }
    }
}

}

#endif // EDYN_PARALLEL_MERGE_MERGE_UTIL_HPP