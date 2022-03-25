#ifndef EDYN_UTIL_REGISTRY_OPERATION_HPP
#define EDYN_UTIL_REGISTRY_OPERATION_HPP

#include <memory>
#include <vector>
#include <entt/entity/registry.hpp>
#include "edyn/comp/dirty.hpp"
#include "edyn/config/config.h"
#include "edyn/parallel/map_child_entity.hpp"
#include "edyn/util/entity_map.hpp"

namespace edyn {

/**
 * @brief Registry operation type.
 */
enum class registry_op_type {
    create, // Create entity.
    destroy, // Destroy entity.
    emplace, // Emplace component.
    replace, // Replace component.
    remove, // Remove component.
    ent_map // Create entity mapping.
};

/**
 * @brief Registry operation for components.
 */
class component_operation {
public:
    virtual ~component_operation() = default;
    virtual void execute(entt::registry &, registry_op_type,
                         const std::vector<entt::entity> &,
                         entity_map &, bool mark_dirty) const = 0;
    virtual entt::id_type get_type_id() const = 0;
    virtual void remap(const entity_map &emap) = 0;
};

template<typename Component>
class component_operation_impl : public component_operation {

    void execute_emplace(entt::registry &registry,
                         const std::vector<entt::entity> &entities,
                         const entity_map &entity_map, bool mark_dirty) const {
        if constexpr(!std::is_empty_v<Component>) {
            EDYN_ASSERT(entities.size() == components.size());
        }

        for (size_t i = 0; i < entities.size(); ++i) {
            auto remote_entity = entities[i];

            if (!entity_map.contains(remote_entity)) {
                continue;
            }

            auto local_entity = entity_map.at(remote_entity);

            if (!registry.valid(local_entity) || registry.all_of<Component>(local_entity)) {
                continue;
            }

            if constexpr(std::is_empty_v<Component>) {
                registry.emplace<Component>(local_entity);
            } else {
                auto comp = components[i];
                internal::map_child_entity(registry, entity_map, comp);
                registry.emplace<Component>(local_entity, comp);
            }

            if (mark_dirty) {
                registry.get_or_emplace<dirty>(local_entity).template created<Component>();
            }
        }
    }

    template<typename T = Component>
    typename std::enable_if_t<!std::is_empty_v<T>>
    execute_replace(entt::registry &registry,
                    const std::vector<entt::entity> &entities,
                    const entity_map &entity_map, bool mark_dirty) const {
        EDYN_ASSERT(entities.size() == components.size());

        for (size_t i = 0; i < entities.size(); ++i) {
            auto remote_entity = entities[i];

            if (!entity_map.contains(remote_entity)) {
                continue;
            }

            auto local_entity = entity_map.at(remote_entity);

            if (!registry.valid(local_entity) || !registry.all_of<Component>(local_entity)) {
                continue;
            }

            auto comp = components[i];
            internal::map_child_entity(registry, entity_map, comp);
            registry.replace<Component>(local_entity, comp);

            if (mark_dirty) {
                registry.get_or_emplace<dirty>(local_entity).template updated<Component>();
            }
        }
    }

    void execute_remove(entt::registry &registry,
                        const std::vector<entt::entity> &entities,
                        const entity_map &entity_map, bool mark_dirty) const {
        for (auto remote_entity : entities) {
            if (!entity_map.contains(remote_entity)) {
                continue;
            }

            auto local_entity = entity_map.at(remote_entity);

            if (!registry.valid(local_entity)) {
                continue;
            }

            registry.remove<Component>(local_entity);

            if (mark_dirty) {
                registry.get_or_emplace<dirty>(local_entity).template destroyed<Component>();
            }
        }
    }

    template<typename T = Component>
    typename std::enable_if_t<std::is_same_v<T, entt::entity>>
    execute_ent_map(const entt::registry &registry,
                    const std::vector<entt::entity> &entities,
                    entity_map &entity_map) const {
        // Component operations are cleverly used to insert entity mappings,
        // thus not requiring a separate means of doing so. Local entities
        // are inserted as components which are related directly to the
        // entity at the same index in the entities array, which is the
        // remote entity.
        EDYN_ASSERT(entities.size() == components.size());

        for (size_t i = 0; i < entities.size(); ++i) {
            auto remote_entity = entities[i];
            auto local_entity = components[i];

            if (registry.valid(local_entity)) {
                entity_map.insert(remote_entity, local_entity);
            }
        }
    }

public:
    static constexpr auto is_empty_type = std::is_empty_v<Component>;
    std::vector<Component> components;

    void execute(entt::registry &registry, registry_op_type op,
                 const std::vector<entt::entity> &entities,
                 entity_map &entity_map, bool mark_dirty) const override {
        switch (op) {
        case registry_op_type::emplace:
            execute_emplace(registry, entities, entity_map, mark_dirty);
            break;
        case registry_op_type::replace:
            if constexpr(!is_empty_type) {
                execute_replace(registry, entities, entity_map, mark_dirty);
            }
            break;
        case registry_op_type::remove:
            execute_remove(registry, entities, entity_map, mark_dirty);
            break;
        case registry_op_type::ent_map:
            if constexpr(std::is_same_v<Component, entt::entity>) {
                execute_ent_map(registry, entities, entity_map);
            }
            break;
        default:
            break;
        }
    }

    entt::id_type get_type_id() const override {
        return entt::type_seq<Component>::value();
    }

    void remap(const entity_map &emap) override {
        if constexpr(!is_empty_type) {
            for (auto &comp : components) {
                internal::map_child_entity_no_validation(emap, comp);
            }
        }
    }
};

/**
 * @brief An operation to replicate contents of one registry into another.
 */
class registry_operation final {

    void execute_create(entt::registry &registry, entity_map &entity_map, bool mark_dirty) const {
        for (auto remote_entity : entities) {
            if (entity_map.contains(remote_entity)) {
                continue;
            }

            auto local_entity = registry.create();
            entity_map.insert(remote_entity, local_entity);

            if (mark_dirty) {
                registry.get_or_emplace<dirty>(local_entity).set_new();
            }
        }
    }

    void execute_destroy(entt::registry &registry, entity_map &entity_map) const {
        for (auto remote_entity : entities) {
            if (!entity_map.contains(remote_entity)) {
                continue;
            }

            auto local_entity = entity_map.at(remote_entity);
            entity_map.erase(remote_entity);

            if (registry.valid(local_entity)) {
                registry.destroy(local_entity);
            }
        }
    }

public:
    registry_op_type operation;
    std::vector<entt::entity> entities;
    std::shared_ptr<component_operation> components;

    void execute(entt::registry &registry, entity_map &entity_map, bool mark_dirty = false) const {
        switch (operation) {
        case registry_op_type::create:
            execute_create(registry, entity_map, mark_dirty);
            break;
        case registry_op_type::destroy:
            execute_destroy(registry, entity_map);
            break;
        default:
            components->execute(registry, operation, entities, entity_map, mark_dirty);
        }
    }

    void remap(const entity_map &emap) {
        for (auto &entity : entities) {
            entity = emap.at(entity);
        }

        components->remap(emap);
    }
};

class registry_operation_collection final {

    template<typename Component, typename Func>
    void for_each_comp(registry_op_type op_type, Func func) const {
        auto type_id = entt::type_seq<Component>::value();

        for (auto &op : operations) {
            if (op.operation == op_type && op.components && op.components->get_type_id() == type_id) {
                if constexpr(std::is_empty_v<Component>) {
                    for (auto entity : op.entities) {
                        func(entity);
                    }
                } else {
                    if (op_type == registry_op_type::remove) {
                        if constexpr(std::is_invocable_v<Func, entt::entity>) {
                            for (auto entity : op.entities) {
                                func(entity);
                            }
                        }
                    } else {
                        auto *components = static_cast<component_operation_impl<Component> *>(op.components.get());
                        EDYN_ASSERT(op.entities.size() == components->components.size());
                        for (size_t i = 0; i < op.entities.size(); ++i) {
                            func(op.entities[i], components->components[i]);
                        }
                    }
                }
            }
        }
    }

    template<typename Func>
    void for_each_entity(registry_op_type op_type,Func func) const {
        for (auto &op : operations) {
            if (op.operation == op_type) {
                for (auto entity : op.entities) {
                    func(entity);
                }
            }
        }
    }

public:
    std::vector<registry_operation> operations;

    void execute(entt::registry &registry,
                 entity_map &entity_map,
                 bool mark_dirty = false) const {
        for (auto &op : operations) {
            op.execute(registry, entity_map, mark_dirty);
        }
    }

    bool empty() const {
        for (auto &op : operations) {
            if (!op.entities.empty()) {
                return false;
            }
        }

        return true;
    }

    template<typename Func>
    void create_for_each(Func func) const {
        for_each_entity(registry_op_type::create, func);
    }

    template<typename Func>
    void destroy_for_each(Func func) const {
        for_each_entity(registry_op_type::destroy, func);
    }

    template<typename... Component, typename Func>
    void emplace_for_each(Func func) const {
        (for_each_comp<Component>(registry_op_type::emplace, func), ...);
    }

    template<typename... Component, typename Func>
    void replace_for_each(Func func) const {
        (for_each_comp<Component>(registry_op_type::replace, func), ...);
    }

    template<typename... Component, typename Func>
    void remove_for_each(Func func) const {
        (for_each_comp<Component>(registry_op_type::remove, func), ...);
    }

    template<typename... Component, typename Func>
    void emplace_for_each([[maybe_unused]] std::tuple<Component...>, Func func) const {
        (for_each_comp<Component>(registry_op_type::emplace, func), ...);
    }

    template<typename... Component, typename Func>
    void replace_for_each([[maybe_unused]] std::tuple<Component...>, Func func) const {
        (for_each_comp<Component>(registry_op_type::replace, func), ...);
    }

    template<typename... Component, typename Func>
    void remove_for_each([[maybe_unused]] std::tuple<Component...>, Func func) const {
        (for_each_comp<Component>(registry_op_type::remove, func), ...);
    }

    template<typename Func>
    void ent_map_for_each(Func func) const {
        for_each_comp<entt::entity>(registry_op_type::ent_map, func);
    }
};

}

#endif // EDYN_UTIL_REGISTRY_OPERATION_HPP
