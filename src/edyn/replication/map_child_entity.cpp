#include "edyn/replication/map_child_entity.hpp"
#include "edyn/config/config.h"

namespace edyn::internal {

static void remove_null_entities_in_sequence(entt::meta_sequence_container &seq) {
    // Move null entities to the end.
    auto end = std::remove_if(seq.begin(), seq.end(), [](auto &&elem) {
        return elem.template cast<entt::entity>() == entt::null;
    });

    // Remove null if container is dynamic.
    while (end != seq.end()) {
        end = seq.erase(end);

        if (!end) { // Not a dynamic container.
            break;
        }
    }
}

void map_child_entity_sequence(const entity_map &emap, entt::meta_sequence_container &seq) {
    if (seq.value_type() != entt::resolve<entt::entity>()) {
        return;
    }

    for (size_t i = 0; i < seq.size(); ++i) {
        auto &entity_ref = seq[i].cast<entt::entity &>();
        auto local_entity = entt::entity{entt::null};

        if (entity_ref != entt::null && emap.contains(entity_ref)) {
            local_entity = emap.at(entity_ref);
        }

        entity_ref = local_entity;
    }

    remove_null_entities_in_sequence(seq);
}

void set_invalid_child_entity_to_null_in_sequence(const entt::registry &registry, entt::meta_sequence_container &seq) {
    if (seq.value_type() != entt::resolve<entt::entity>()) {
        return;
    }

    for (size_t i = 0; i < seq.size(); ++i) {
        auto &entity_ref = seq[i].cast<entt::entity &>();

        if (!registry.valid(entity_ref)) {
            entity_ref = entt::null;
        }
    }

    remove_null_entities_in_sequence(seq);
}

void map_child_entity_associative(const entity_map &emap, entt::meta_associative_container &map) {
    EDYN_ASSERT(map.mapped_type() == entt::resolve<entt::entity>());
    std::vector<entt::meta_any> erase_keys;

    for (auto pair : map) {
        auto &entity_ref = pair.second.cast<entt::entity &>();

        if (entity_ref != entt::null && emap.contains(entity_ref)) {
            entity_ref = emap.at(entity_ref);
        } else {
            erase_keys.push_back(pair.first);
        }
    }

    for (auto &key : erase_keys) {
        map.erase(key);
    }
}

}
