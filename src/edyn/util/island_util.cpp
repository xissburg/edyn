#include "edyn/util/island_util.hpp"
#include "edyn/comp.hpp"
#include "edyn/parallel/island_worker.hpp"
#include "edyn/dynamics/world.hpp"
#include "edyn/parallel/message.hpp"
#include "edyn/comp/island.hpp"
#include <entt/entt.hpp>

namespace edyn {

entity_set get_island_node_children(const entt::registry &registry, entt::entity node_entity) {
    EDYN_ASSERT(registry.has<island_node_parent>(node_entity));

    entity_set children;
    std::vector<entt::entity> parents_to_visit;
    parents_to_visit.push_back(node_entity);

    while (!parents_to_visit.empty()) {
        auto parent_entity = parents_to_visit.back();
        parents_to_visit.pop_back();

        auto &parent = registry.get<island_node_parent>(parent_entity);
        for (auto child : parent.children) {
            children.insert(child);

            if (registry.has<island_node_parent>(child)) {
                parents_to_visit.push_back(child);
            }
        }
    }

    return children;
}

}