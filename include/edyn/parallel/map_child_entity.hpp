#ifndef EDYN_PARALLEL_MAP_CHILD_ENTITY_HPP
#define EDYN_PARALLEL_MAP_CHILD_ENTITY_HPP

#include <entt/meta/meta.hpp>
#include <entt/meta/resolve.hpp>
#include <entt/meta/container.hpp>
#include <entt/entity/registry.hpp>
#include "edyn/util/entity_map.hpp"

namespace edyn::internal {

void map_child_entity_sequence(const entity_map &emap, entt::meta_sequence_container &seq);
void set_invalid_child_entity_to_null_in_sequence(const entt::registry &registry, entt::meta_sequence_container &seq);

template<typename Value>
void map_child_entity_meta(const entity_map &emap, const entt::meta_type &meta_type, Value &value) {
    auto range = meta_type.data();

    for (entt::meta_data data : range) {
        if (data.type() == entt::resolve<entt::entity>()) {
            // If the member is an entity, assign the local value or null if
            // it's unavailable.
            auto remote_entity = data.get(entt::meta_handle(value)).cast<entt::entity>();
            auto local_entity = entt::entity{entt::null};

            if (emap.contains(remote_entity)) {
                local_entity = emap.at(remote_entity);
            }

            data.set(entt::meta_handle(value), local_entity);
        } else if (data.type().is_sequence_container()) {
            auto seq = data.get(entt::meta_handle(value)).as_sequence_container();

            if (seq.value_type() == entt::resolve<entt::entity>()) {
                // If it's a sequence of entities, map all to local.
                map_child_entity_sequence(emap, seq);
            } else if (auto child_type = entt::resolve(seq.value_type().id()); child_type) {
                // If it's a sequence of something else, call this function
                // recursively to map the entities contained in it.
                for (size_t i = 0; i < seq.size(); ++i) {
                    auto val = seq[i];
                    map_child_entity_meta(emap, seq[i].type(), val);
                }
            }
        } else if (auto child_type = entt::resolve(data.id()); child_type) {
            // If the member is another type registered in entt::meta, call this
            // function recursively to map the entities contained in it.
            auto child_value = data.get(entt::meta_handle(value));
            map_child_entity_meta(emap, child_type, child_value);
        }
    }
}

template<typename Value>
void set_invalid_child_entity_to_null_meta(const entt::registry &registry, const entt::meta_type &meta_type, Value &value) {
    auto range = meta_type.data();

    for (entt::meta_data data : range) {
        if (data.type() == entt::resolve<entt::entity>()) {
            auto entity = data.get(entt::meta_handle(value)).cast<entt::entity>();

            if (!registry.valid(entity)) {
                data.set(entt::meta_handle(value), entt::entity{entt::null});
            }
        } else if (data.type().is_sequence_container()) {
            auto seq = data.get(entt::meta_handle(value)).as_sequence_container();

            if (seq.value_type() == entt::resolve<entt::entity>()) {
                set_invalid_child_entity_to_null_in_sequence(registry, seq);
            } else if (auto child_type = entt::resolve(seq.value_type().id()); child_type) {
                // If it's a sequence of something else, call this function
                // recursively to set other invalid entities to null;
                for (size_t i = 0; i < seq.size(); ++i) {
                    auto val = seq[i];
                    set_invalid_child_entity_to_null_meta(registry, seq[i].type(), val);
                }
            }
        } else if (auto child_type = entt::resolve(data.id()); child_type) {
            // If the member is another type registered in entt::meta, call this
            // function recursively to set the invalid entities contained in it
            // to null.
            auto child_value = data.get(entt::meta_handle(value));
            set_invalid_child_entity_to_null_meta(registry, child_type, child_value);
        }
    }
}

template<typename Component>
void map_child_entity(const entt::registry &registry, const entity_map &emap, Component &component) {
    if (auto meta_type = entt::resolve<Component>(); meta_type) {
        map_child_entity_meta(emap, meta_type, component);
        set_invalid_child_entity_to_null_meta(registry, meta_type, component);
    }
}

template<typename Component>
void map_child_entity_no_validation(const entity_map &emap, Component &component) {
    if (auto meta_type = entt::resolve<Component>(); meta_type) {
        map_child_entity_meta(emap, meta_type, component);
    }
}

}

#endif // EDYN_PARALLEL_MAP_CHILD_ENTITY_HPP
