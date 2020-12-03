#ifndef EDYN_PARALLEL_MERGE_MERGE_UTIL_HPP
#define EDYN_PARALLEL_MERGE_MERGE_UTIL_HPP

#include <array>
#include <unordered_set>
#include <entt/entity/registry.hpp>
#include "edyn/parallel/merge/merge_component.hpp"
#include "edyn/util/entity_map.hpp"

namespace edyn {

std::vector<std::unordered_set<entt::entity>> map_split_to_local(const merge_context &ctx);

template<merge_type MergeType, typename Component>
void merge_unordered_set(const Component *old_comp, 
                         Component &new_comp, 
                         std::unordered_set<entt::entity> Component:: *member, 
                         const merge_context &ctx) {

    std::unordered_set<entt::entity> entities;

    for (auto remote_entity : new_comp.*member) {
        if (!ctx.map->has_rem(remote_entity)) continue;
        auto local_entity = ctx.map->remloc(remote_entity);
        if (!ctx.registry->valid(local_entity)) continue;
        entities.insert(local_entity);
    }

    new_comp.*member = entities;

    if constexpr(MergeType == merge_type::updated) {
        // Reinsert entities from old which are still valid and haven't been
        // moved to another island due to a split.
        auto split = map_split_to_local(ctx);

        for (auto old_entity : old_comp->*member) {
            if (!ctx.registry->valid(old_entity)) continue;

            if ((new_comp.*member).count(old_entity)) continue;

            bool moved = false;
            // Ignore entity if it has moved to another island due to a split.
            for (auto &connected : split) {
                if (connected.count(old_entity)) {
                    moved = true;
                    break;
                }
            }

            if (moved) continue;

            (new_comp.*member).insert(old_entity);
        }
    }
}

template<merge_type MergeType, typename Component, std::size_t N>
void merge_array(const Component *old_comp, 
                 Component &new_comp, 
                 std::array<entt::entity, N> Component:: *member, 
                 const merge_context &ctx) {
    
    for (auto &entity : new_comp.*member) {
        if (entity == entt::null) continue;

        if (ctx.map->has_rem(entity)) {
            auto local_entity = ctx.map->remloc(entity);
            if (ctx.registry->valid(local_entity)) {
                entity = local_entity;
            } else {
                entity = entt::null;
            }
        } else {
            entity = entt::null;
        }
    }

    if constexpr(MergeType == merge_type::updated) {
        // Reinsert entities from old which are still valid and haven't been
        // moved to another island due to a split.
        auto split = map_split_to_local(ctx);

        for (auto &old_entity : old_comp->*member) {
            if (!ctx.registry->valid(old_entity)) continue;
            
            auto found_it = std::find((new_comp.*member).begin(), (new_comp.*member).end(), old_entity);
            if (found_it != (new_comp.*member).end()) continue;

            auto moved = false;
            for (auto &connected : split) {
                if (connected.count(old_entity)) {
                    moved = true;
                    break;
                }
            }
            if (moved) continue;
            
            // Find a null entry to replace with.
            auto null_it = std::find((new_comp.*member).begin(), (new_comp.*member).end(), entt::entity{entt::null});
            EDYN_ASSERT(null_it != (new_comp.*member).end());
            *null_it = old_entity;
        }
    }

    // Move null entities to the end of the array.
    std::remove((new_comp.*member).begin(), (new_comp.*member).end(), entt::entity{entt::null});
}

}

#endif // EDYN_PARALLEL_MERGE_MERGE_UTIL_HPP