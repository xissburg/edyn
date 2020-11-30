#include "edyn/parallel/merge/merge_util.hpp"
#include "edyn/parallel/registry_delta.hpp"

namespace edyn {

std::vector<std::unordered_set<entt::entity>> map_split_to_local(const merge_context &ctx) {
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

    return split;
}

}