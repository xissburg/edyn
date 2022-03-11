#ifndef EDYN_PARALLEL_IMPORT_CHILD_ENTITY_HPP
#define EDYN_PARALLEL_IMPORT_CHILD_ENTITY_HPP

#include <entt/meta/meta.hpp>
#include <entt/meta/resolve.hpp>
#include <entt/meta/container.hpp>
#include <entt/entity/registry.hpp>
#include <pstl/glue_algorithm_defs.h>
#include "edyn/util/entity_map.hpp"

namespace edyn::internal {

void import_child_entity_sequence(entt::registry &registry, const entity_map &emap, entt::meta_sequence_container &seq);

template<typename Value>
void import_child_entity_meta(entt::registry &registry, const entity_map &emap, const entt::meta_type &meta_type, Value &value) {
    auto range = meta_type.data();

    for (entt::meta_data data : range) {
        if (data.type() == entt::resolve<entt::entity>()) {
            // If the member is an entity, assign the local value or null if
            // it's unavailable or invalid.
            auto remote_entity = data.get(entt::meta_handle(value)).cast<entt::entity>();
            auto local_entity = entt::entity{entt::null};

            if (emap.has_rem(remote_entity)) {
                local_entity = emap.remloc(remote_entity);

                if (!registry.valid(local_entity)) {
                    local_entity = entt::null;
                }
            }

            data.set(entt::meta_handle(value), local_entity);
        } else if (data.type().is_sequence_container()) {
            auto seq = data.get(entt::meta_handle(value)).as_sequence_container();

            if (seq.value_type() == entt::resolve<entt::entity>()) {
                // If it's a sequence of entities, map all to local.
                import_child_entity_sequence(registry, emap, seq);
            } else if (auto child_type = entt::resolve(seq.value_type().id()); child_type) {
                // If it's a sequence of something else, call this function
                // recursively to map the entities contained in it.
                for (size_t i = 0; i < seq.size(); ++i) {
                    auto val = seq[i];
                    import_child_entity_meta(registry, emap, seq[i].type(), val);
                }
            }
        } else if (auto child_type = entt::resolve(data.id()); child_type) {
            // If the member is another type registered in entt::meta, call this
            // function recursively to map the entities contained in it.
            auto child_value = data.get(entt::meta_handle(value));
            import_child_entity_meta(registry, emap, child_type, child_value);
        }
    }
}

template<typename Component>
void import_child_entity(entt::registry &registry, const entity_map &emap, Component &component) {
    if (auto meta_type = entt::resolve<Component>(); meta_type) {
        import_child_entity_meta(registry, emap, meta_type, component);
    }
}

}

#endif // EDYN_PARALLEL_IMPORT_CHILD_ENTITY_HPP
