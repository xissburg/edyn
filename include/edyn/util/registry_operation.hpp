#ifndef EDYN_UTIL_REGISTRY_OPERATION_HPP
#define EDYN_UTIL_REGISTRY_OPERATION_HPP

#include <map>
#include <memory>
#include <vector>
#include <entt/entity/registry.hpp>
#include "edyn/comp/dirty.hpp"
#include "edyn/config/config.h"
#include "edyn/parallel/import_child_entity.hpp"

namespace edyn {

enum class registry_op_type {
    create,
    destroy,
    emplace,
    replace,
    remove,
    ent_map
};

class component_operation {
public:
    virtual ~component_operation();
    virtual void execute(entt::registry &, registry_op_type,
                         const std::vector<entt::entity> &,
                         const std::map<entt::entity, entt::entity> &,
                         bool mark_dirty) const = 0;
    virtual entt::id_type get_type_id() const = 0;
};

template<typename Component>
class component_operation_impl : public component_operation {

    void execute_emplace(entt::registry &registry,
                         const std::vector<entt::entity> &entities,
                         const std::map<entt::entity, entt::entity> &entity_map,
                         bool mark_dirty) const {
        if constexpr(!std::is_empty_v<Component>) {
            EDYN_ASSERT(entities.size() == components.size());
        }

        for (size_t i = 0; i < entities.size(); ++i) {
            auto remote_entity = entities[i];

            if (!entity_map.count(remote_entity)) {
                continue;
            }

            auto local_entity = entity_map.at(remote_entity);

            if (registry.any_of<Component>(local_entity)) {
                continue;
            }

            if constexpr(std::is_empty_v<Component>) {
                registry.emplace<Component>(local_entity);
            } else {
                auto comp = components[i];
                internal::import_child_entity(registry, entity_map, comp);
                registry.emplace<Component>(local_entity, comp);
            }

            if (mark_dirty) {
                registry.get_or_emplace<dirty>(local_entity).template created<Component>();
            }
        }
    }

    template<std::enable_if_t<!std::is_empty_v<Component>, bool> = true>
    void execute_replace(entt::registry &registry,
                         const std::vector<entt::entity> &entities,
                         const std::map<entt::entity, entt::entity> &entity_map,
                         bool mark_dirty) const {
        EDYN_ASSERT(entities.size() == components.size());

        for (size_t i = 0; i < entities.size(); ++i) {
            auto remote_entity = entities[i];

            if (!entity_map.count(remote_entity)) {
                continue;
            }

            auto local_entity = entity_map.at(remote_entity);

            if (registry.any_of<Component>(local_entity)) {
                continue;
            }

            auto comp = components[i];
            internal::import_child_entity(registry, entity_map, comp);
            registry.replace<Component>(local_entity, comp);

            if (mark_dirty) {
                registry.get_or_emplace<dirty>(local_entity).template updated<Component>();
            }
        }
    }

    void execute_remove(entt::registry &registry,
                        const std::vector<entt::entity> &entities,
                        const std::map<entt::entity, entt::entity> &entity_map,
                        bool mark_dirty) {
        for (auto remote_entity : entities) {
            if (!entity_map.count(remote_entity)) {
                continue;
            }

            auto local_entity = entity_map.at(remote_entity);
            registry.remove<Component>(local_entity);

            if (mark_dirty) {
                registry.get_or_emplace<dirty>(local_entity).template destroyed<Component>();
            }
        }
    }

public:
    static constexpr auto is_empty_type = std::is_empty_v<Component>;
    std::enable_if_t<!is_empty_type, std::vector<Component>> components;

    void execute(entt::registry &registry, registry_op_type op,
                 const std::vector<entt::entity> &entities,
                 const std::map<entt::entity, entt::entity> &entity_map,
                 bool mark_dirty) const override {
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
        default:
            break;
        }
    }

    entt::id_type get_type_id() const {
        return entt::type_id<Component>().seq();
    }
};

class registry_operation final {
    void execute_create(entt::registry &registry,
                        std::map<entt::entity, entt::entity> &entity_map,
                        bool mark_dirty) const {
        for (auto remote_entity : entities) {
            if (entity_map.count(remote_entity)) {
                continue;
            }

            auto local_entity = registry.create();
            entity_map[remote_entity] = local_entity;

            if (mark_dirty) {
                registry.get_or_emplace<dirty>(local_entity).set_new();
            }
        }
    }

    void execute_destroy(entt::registry &registry,
                         std::map<entt::entity, entt::entity> &entity_map) const {
        for (auto remote_entity : entities) {
            if (!entity_map.count(remote_entity)) {
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

    void execute(entt::registry &registry,
                 std::map<entt::entity, entt::entity> &entity_map,
                 bool mark_dirty) const {
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
};

class registry_operation_collection final {

    template<typename Component, typename Func>
    void for_each_comp(registry_op_type op_type, Func func) const {
        auto type_id = entt::type_id<Component>().seq();

        for (auto &op : operations) {
            if (op.operation == op_type && op.components && op.components->get_type_id() == type_id) {
                if constexpr(std::is_empty_v<Component>) {
                    for (auto entity : op.entities) {
                        func(entity);
                    }
                } else {
                    if (op_type == registry_op_type::remove) {
                        for (auto entity : op.entities) {
                            func(entity);
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
                 std::map<entt::entity, entt::entity> &entity_map,
                 bool mark_dirty) const {
        for (auto &op : operations) {
            op.execute(registry, entity_map, mark_dirty);
        }
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
};

class registry_operations_builder {
    template<typename Component>
    registry_operation & find_or_create_component_operation(registry_op_type op_type) {
        EDYN_ASSERT(op_type == registry_op_type::emplace ||
                    op_type == registry_op_type::replace ||
                    op_type == registry_op_type::remove);

        auto type_id = entt::type_id<Component>().seq();

        for (auto &op : operations) {
            if (op.operation == op_type && op.components && op.components->get_type_id() == type_id) {
                return op;
            }
        }

        auto &op = operations.emplace_back();
        op.operation = op_type;
        op.components = std::make_shared<component_operation_impl<Component>>();
        return op;
    }

    registry_operation & find_or_create_entity_operation(registry_op_type op_type) {
        EDYN_ASSERT(op_type == registry_op_type::create ||
                    op_type == registry_op_type::destroy);

        for (auto &op : operations) {
            if (op.operation == op_type) {
                return op;
            }
        }

        auto &op = operations.emplace_back();
        op.operation = op_type;
        return op;
    }

public:
    template<typename It>
    void create(It first, It last) {
        auto &op = find_or_create_entity_operation(registry_op_type::create);
        op.entities.insert(op.entities.end(), first, last);
    }

    template<typename Component, typename It>
    void replace(const entt::registry &registry, It first, It last) {
        auto &op = find_or_create_component_operation<Component>(registry_op_type::replace);
        auto view = registry.view<Component>();

        for (; first != last; ++first) {
            auto entity = *first;
            op.entities.push_back(entity);

            if constexpr(!std::is_empty_v<Component>) {
                auto [comp] = view.get(entity);
                auto *components = static_cast<component_operation_impl<Component> *>(op.components.get());
                components->components.push_back(comp);
            }
        }
    }

    template<typename Component>
    void replace(const entt::registry &registry) {
        auto &op = find_or_create_component_operation<Component>(registry_op_type::replace);
        auto view = registry.view<Component>();

        for (auto entity : view) {
            op.entities.push_back(entity);

            if constexpr(!std::is_empty_v<Component>) {
                auto [comp] = view.get(entity);
                auto *components = static_cast<component_operation_impl<Component> *>(op.components.get());
                components->components.push_back(comp);
            }
        }
    }

    void replace_type_id(const entt::registry &registry, entt::entity entity, entt::id_type id) {

    }

    auto finish() { return std::move(operations); }

private:
    std::vector<registry_operation> operations;
};

}

#endif // EDYN_UTIL_REGISTRY_OPERATION_HPP
