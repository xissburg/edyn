#include "edyn/parallel/import_child_entity.hpp"

namespace edyn::internal {

void import_child_entity_sequence(entt::registry &registry, const entity_map &emap, entt::meta_sequence_container &seq) {
    if (seq.value_type() != entt::resolve<entt::entity>()) {
        return;
    }

    for (size_t i = 0; i < seq.size(); ++i) {
        auto &entity_ref = seq[i].cast<entt::entity &>();
        auto local_entity = entt::entity{entt::null};

        if (emap.has_rem(entity_ref)) {
            local_entity = emap.remloc(entity_ref);

            if (!registry.valid(local_entity)) {
                local_entity = entt::null;
            }
        }

        entity_ref = local_entity;
    }

    // Move null entities to the end.
    auto end = std::remove_if(seq.begin(), seq.end(), [] (auto &&elem) {
        return elem.template cast<entt::entity>() == entt::null;
    });

    // Remove null if container is dynamic.
    while (end != seq.end()) {
        auto pair = seq.erase(end);
        if (!pair.second) { // Not a dynamic container.
            break;
        }
    }
}

}