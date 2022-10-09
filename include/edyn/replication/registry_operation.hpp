#ifndef EDYN_REPLICATION_REGISTRY_OPERATION_HPP
#define EDYN_REPLICATION_REGISTRY_OPERATION_HPP

#include <memory>
#include <type_traits>
#include <vector>
#include <entt/entity/registry.hpp>
#include "edyn/config/config.h"
#include "edyn/core/entity_pair.hpp"
#include "edyn/replication/map_child_entity.hpp"
#include "edyn/replication/entity_map.hpp"
#include "edyn/comp/merge_component.hpp"

namespace edyn {

/**
 * @brief Registry operation for components.
 */
class component_operation {
public:
    virtual ~component_operation() = default;
    virtual void execute(entt::registry &, entity_map &) const = 0;
    virtual void execute(entt::registry &) const = 0;
    virtual entt::id_type get_type_id() const = 0;

    virtual void remap(const entity_map &emap) {
        for (auto &remote_entity : entities) {
            auto local_entity = emap.at(remote_entity);
            remote_entity = local_entity;
        }
    }

    bool empty() const {
        return entities.empty();
    }

    std::vector<entt::entity> entities;
};

template<typename Component>
class component_operation_emplace : public component_operation {
public:
    static constexpr auto is_empty_type = std::is_empty_v<Component>;
    std::vector<Component> components;

    void execute(entt::registry &registry, entity_map &entity_map) const override {
        if constexpr(!is_empty_type) {
            EDYN_ASSERT(entities.size() == components.size());
        }

        for (unsigned i = 0; i < entities.size(); ++i) {
            auto remote_entity = entities[i];

            if (!entity_map.contains(remote_entity)) {
                continue;
            }

            auto local_entity = entity_map.at(remote_entity);

            if (!registry.valid(local_entity) || registry.all_of<Component>(local_entity)) {
                continue;
            }

            if constexpr(is_empty_type) {
                registry.emplace<Component>(local_entity);
            } else {
                auto comp = components[i];
                internal::map_child_entity(registry, entity_map, comp);
                registry.emplace<Component>(local_entity, comp);
            }
        }
    }

    void execute(entt::registry &registry) const override {
        if constexpr(!is_empty_type) {
            EDYN_ASSERT(entities.size() == components.size());
        }

        for (unsigned i = 0; i < entities.size(); ++i) {
            auto entity = entities[i];

            if (!registry.valid(entity) || registry.all_of<Component>(entity)) {
                continue;
            }

            if constexpr(is_empty_type) {
                registry.emplace<Component>(entity);
            } else {
                auto comp = components[i];
                registry.emplace<Component>(entity, comp);
            }
        }
    }

    entt::id_type get_type_id() const override {
        return entt::type_index<Component>::value();
    }

    void remap(const entity_map &emap) override {
        component_operation::remap(emap);

        if constexpr(!is_empty_type) {
            for (auto &comp : components) {
                internal::map_child_entity_no_validation(emap, comp);
            }
        }
    }
};

template<typename Component>
class component_operation_replace : public component_operation {
public:
    std::vector<Component> components;

    void execute(entt::registry &registry, entity_map &entity_map) const override {
        if constexpr(!std::is_empty_v<Component>) {
            EDYN_ASSERT(entities.size() == components.size());

            for (unsigned i = 0; i < entities.size(); ++i) {
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
                registry.patch<Component>(local_entity, [&](auto &&current) {
                    merge_component(current, comp);
                });
            }
        }
    }

    void execute(entt::registry &registry) const override {
        if constexpr(!std::is_empty_v<Component>) {
            EDYN_ASSERT(entities.size() == components.size());

            for (unsigned i = 0; i < entities.size(); ++i) {
                auto entity = entities[i];

                if (!registry.valid(entity) || !registry.all_of<Component>(entity)) {
                    continue;
                }

                auto comp = components[i];
                registry.patch<Component>(entity, [&](auto &&current) {
                    merge_component(current, comp);
                });
            }
        }
    }

    entt::id_type get_type_id() const override {
        return entt::type_index<Component>::value();
    }

    void remap(const entity_map &emap) override {
        component_operation::remap(emap);

        for (auto &comp : components) {
            internal::map_child_entity_no_validation(emap, comp);
        }
    }
};

template<typename Component>
class component_operation_remove : public component_operation {
public:
    void execute(entt::registry &registry, entity_map &entity_map) const override {
        for (auto remote_entity : entities) {
            if (!entity_map.contains(remote_entity)) {
                continue;
            }

            auto local_entity = entity_map.at(remote_entity);

            if (!registry.valid(local_entity)) {
                continue;
            }

            registry.remove<Component>(local_entity);
        }
    }

    void execute(entt::registry &registry) const override {
        for (auto entity : entities) {
            if (registry.valid(entity)) {
                registry.remove<Component>(entity);
            }
        }
    }

    entt::id_type get_type_id() const override {
        return entt::type_index<Component>::value();
    }
};

/**
 * @brief An operation to replicate contents of one registry into another.
 */
class registry_operation final {

    void execute_create(entt::registry &registry, entity_map &entity_map) const {
        for (auto remote_entity : create_entities) {
            if (entity_map.contains(remote_entity)) {
                continue;
            }

            auto local_entity = registry.create();
            entity_map.insert(remote_entity, local_entity);
        }
    }

    void execute_destroy(entt::registry &registry, entity_map &entity_map) const {
        for (auto remote_entity : destroy_entities) {
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

    void execute_map_entities(const entt::registry &registry,
                              entity_map &entity_map) const {
        for (auto [remote_entity, local_entity] : map_entities) {
            if (registry.valid(local_entity)) {
                if (entity_map.contains(remote_entity)) {
                    EDYN_ASSERT(entity_map.at(remote_entity) == local_entity);
                } else {
                    entity_map.insert(remote_entity, local_entity);
                }
            }
        }
    }

    template<typename Component, typename Func>
    void for_each_emplace(Func func) const {
        auto found_it = std::find_if(emplace_components.begin(), emplace_components.end(),
                                     [](auto &&op) { return op->get_type_id() == entt::type_index<Component>::value(); });
        if (found_it != emplace_components.end()) {
            auto *typed_op = static_cast<const component_operation_emplace<Component> *>(found_it->get());
            for (unsigned i = 0; i < typed_op->entities.size(); ++i) {
                auto entity = typed_op->entities[i];
                if constexpr(std::is_empty_v<Component>) {
                    func(entity);
                } else {
                    auto &comp = typed_op->components[i];
                    func(entity, comp);
                }
            }
        }
    }

    template<typename Component, typename Func>
    void for_each_replace(Func func) const {
        auto found_it = std::find_if(replace_components.begin(), replace_components.end(),
                                     [](auto &&op) { return op->get_type_id() == entt::type_index<Component>::value(); });
        if (found_it != replace_components.end()) {
            auto *typed_op = static_cast<const component_operation_replace<Component> *>(found_it->get());
            for (unsigned i = 0; i < typed_op->entities.size(); ++i) {
                auto entity = typed_op->entities[i];

                if constexpr(std::is_invocable_v<Func, entt::entity>) {
                    func(entity);
                } else {
                    if constexpr(std::is_empty_v<Component>) {
                        func(entity, Component{});
                    } else {
                        func(entity, typed_op->components[i]);
                    }
                }
            }
        }
    }

    template<typename Component, typename Func>
    void for_each_remove(Func func) const {
        auto found_it = std::find_if(remove_components.begin(), remove_components.end(),
                                     [](auto &&op) { return op->get_type_id() == entt::type_index<Component>::value(); });
        if (found_it != remove_components.end()) {
            auto *typed_op = static_cast<const component_operation_remove<Component> *>(found_it->get());
            for (unsigned i = 0; i < typed_op->entities.size(); ++i) {
                auto entity = typed_op->entities[i];
                func(entity);
            }
        }
    }

public:
    registry_operation() = default;
    registry_operation(registry_operation &) = delete;
    registry_operation(registry_operation &&) = default;
    registry_operation & operator=(registry_operation &) = delete;
    registry_operation & operator=(registry_operation &&) = default;

    std::vector<entt::entity> create_entities;
    std::vector<entt::entity> destroy_entities;
    entity_pair_vector map_entities;

    std::vector<std::unique_ptr<component_operation>> emplace_components;
    std::vector<std::unique_ptr<component_operation>> replace_components;
    std::vector<std::unique_ptr<component_operation>> remove_components;

    void execute(entt::registry &registry, entity_map &entity_map) const {
        execute_map_entities(registry, entity_map);
        execute_create(registry, entity_map);

        for (auto &op : emplace_components) {
            op->execute(registry, entity_map);
        }

        for (auto &op : replace_components) {
            op->execute(registry, entity_map);
        }

        for (auto &op : remove_components) {
            op->execute(registry, entity_map);
        }

        execute_destroy(registry, entity_map);
    }

    void execute(entt::registry &registry) const {
        for (auto &op : emplace_components) {
            op->execute(registry);
        }

        for (auto &op : replace_components) {
            op->execute(registry);
        }

        for (auto &op : remove_components) {
            op->execute(registry);
        }
    }

    void remap(const entity_map &emap) {
        for (auto &entity : create_entities) {
            entity = emap.at(entity);
        }

        for (auto &entity : destroy_entities) {
            entity = emap.at(entity);
        }

        for (auto &[remote_entity, local_entity] : map_entities) {
            local_entity = emap.at(remote_entity);
            remote_entity = emap.at_local(local_entity);
        }

        for (auto &op : emplace_components) {
            op->remap(emap);
        }

        for (auto &op : replace_components) {
            op->remap(emap);
        }

        for (auto &op : remove_components) {
            op->remap(emap);
        }
    }

    bool empty() const {
        return create_entities.empty() &&
               destroy_entities.empty() &&
               map_entities.empty() &&
               emplace_components.empty() &&
               replace_components.empty() &&
               remove_components.empty();
    }

    template<typename Func>
    void create_for_each(Func func) const {
        for (auto entity : create_entities) {
            func(entity);
        }
    }

    template<typename Func>
    void destroy_for_each(Func func) const {
        for (auto entity : destroy_entities) {
            func(entity);
        }
    }

    template<typename... Component, typename Func>
    void emplace_for_each(Func func) const {
        (for_each_emplace<Component>(func), ...);
    }

    template<typename... Component, typename Func>
    void replace_for_each(Func func) const {
        (for_each_replace<Component>(func), ...);
    }

    template<typename... Component, typename Func>
    void remove_for_each(Func func) const {
        (for_each_remove<Component>(func), ...);
    }

    template<typename... Component, typename Func>
    void emplace_for_each([[maybe_unused]] std::tuple<Component...>, Func func) const {
        (for_each_emplace<Component>(func), ...);
    }

    template<typename... Component, typename Func>
    void replace_for_each([[maybe_unused]] std::tuple<Component...>, Func func) const {
        (for_each_replace<Component>(func), ...);
    }

    template<typename... Component, typename Func>
    void remove_for_each([[maybe_unused]] std::tuple<Component...>, Func func) const {
        (for_each_remove<Component>(func), ...);
    }

    template<typename Func>
    void ent_map_for_each(Func func) const {
        for (auto [remote_entity, local_entity] : map_entities) {
            func(remote_entity, local_entity);
        }
    }
};

}

#endif // EDYN_REPLICATION_REGISTRY_OPERATION_HPP
