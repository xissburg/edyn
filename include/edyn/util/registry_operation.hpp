#ifndef EDYN_UTIL_REGISTRY_OPERATION_HPP
#define EDYN_UTIL_REGISTRY_OPERATION_HPP

#include <memory>
#include <vector>
#include <entt/entity/registry.hpp>
#include "edyn/comp/dirty.hpp"
#include "edyn/config/config.h"
#include "edyn/parallel/import_child_entity.hpp"
#include "edyn/util/entity_map.hpp"

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
    virtual ~component_operation() = default;
    virtual void execute(entt::registry &, registry_op_type,
                         const std::vector<entt::entity> &,
                         entity_map &, bool mark_dirty) const = 0;
    virtual entt::id_type get_type_id() const = 0;
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

            if (!entity_map.count(remote_entity)) {
                continue;
            }

            auto local_entity = entity_map.at(remote_entity);

            if (registry.all_of<Component>(local_entity)) {
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
                         const entity_map &entity_map, bool mark_dirty) const {
        EDYN_ASSERT(entities.size() == components.size());

        for (size_t i = 0; i < entities.size(); ++i) {
            auto remote_entity = entities[i];

            if (!entity_map.count(remote_entity)) {
                continue;
            }

            auto local_entity = entity_map.at(remote_entity);

            if (!registry.all_of<Component>(local_entity)) {
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
                        const entity_map &entity_map, bool mark_dirty) const {
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

    template<typename T = Component>
    typename std::enable_if_t<std::is_same_v<T, entt::entity>>
    execute_ent_map(const entt::registry &registry,
                         const std::vector<entt::entity> &entities,
                         entity_map &entity_map) const {
        EDYN_ASSERT(entities.size() == components.size());

        for (size_t i = 0; i < entities.size(); ++i) {
            auto remote_entity = entities[i];
            auto local_entity = components[i];

            if (registry.valid(local_entity)) {
                entity_map[remote_entity] = local_entity;
            }
        }
    }

public:
    static constexpr auto is_empty_type = std::is_empty_v<Component>;
    std::enable_if_t<!is_empty_type, std::vector<Component>> components;

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
};

class registry_operation final {
    void execute_create(entt::registry &registry, entity_map &entity_map, bool mark_dirty) const {
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

    void execute_destroy(entt::registry &registry, entity_map &entity_map) const {
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
                 entity_map &entity_map,
                 bool mark_dirty = false) const {
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
                    op_type == registry_op_type::remove ||
                    op_type == registry_op_type::ent_map);

        auto type_id = entt::type_seq<Component>::value();

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

    template<typename Component, typename ViewType>
    void insert_components(const ViewType &view, registry_operation &op, entt::entity entity) {
        op.entities.push_back(entity);

        if constexpr(!std::is_empty_v<Component>) {
            if (op.operation != registry_op_type::remove) {
                auto [comp] = view.get(entity);
                auto *components = static_cast<component_operation_impl<Component> *>(op.components.get());
                components->components.push_back(comp);
            }
        }
    }

    template<typename Component, typename It>
    void insert_components(const entt::registry &registry, registry_op_type op_type, It first, It last, bool check = false) {
        EDYN_ASSERT(op_type == registry_op_type::emplace ||
                    op_type == registry_op_type::replace ||
                    op_type == registry_op_type::remove);

        auto &op = find_or_create_component_operation<Component>(op_type);
        auto view = registry.view<Component>();

        for (; first != last; ++first) {
            auto entity = *first;
            if (!check || view.contains(entity)) {
                insert_components<Component>(view, op, entity);
            }
        }
    }

public:
    virtual ~registry_operations_builder() = default;

    template<typename It>
    void create(It first, It last) {
        auto &op = find_or_create_entity_operation(registry_op_type::create);
        op.entities.insert(op.entities.end(), first, last);
    }

    void create(entt::entity entity) {
        auto &op = find_or_create_entity_operation(registry_op_type::create);
        op.entities.push_back(entity);
    }

    template<typename It>
    void destroy(It first, It last) {
        auto &op = find_or_create_entity_operation(registry_op_type::destroy);
        op.entities.insert(op.entities.end(), first, last);
    }

    void destroy(entt::entity entity) {
        auto &op = find_or_create_entity_operation(registry_op_type::destroy);
        op.entities.push_back(entity);
    }

    template<typename Component, typename It>
    void emplace(const entt::registry &registry, It first, It last, bool check = false) {
        insert_components<Component>(registry, registry_op_type::emplace, first, last, check);
    }

    template<typename Component>
    void emplace(const entt::registry &registry) {
        auto view = registry.view<Component>();
        emplace<Component>(registry, view.begin(), view.end());
    }

    template<typename Component>
    void emplace(const entt::registry &registry, entt::entity entity) {
        auto &op = find_or_create_component_operation<Component>(registry_op_type::emplace);
        auto view = registry.view<Component>();
        EDYN_ASSERT(registry.all_of<Component>(entity));
        insert_components<Component>(view, op, entity);
    }

    template<typename Component, typename It>
    void replace(const entt::registry &registry, It first, It last, bool check = false) {
        insert_components<Component>(registry, registry_op_type::replace, first, last, check);
    }

    template<typename Component>
    void replace(const entt::registry &registry) {
        auto view = registry.view<Component>();
        replace<Component>(registry, view.begin(), view.end());
    }

    template<typename Component>
    void replace(const entt::registry &registry, entt::entity entity) {
        auto &op = find_or_create_component_operation<Component>(registry_op_type::replace);
        auto view = registry.view<Component>();
        EDYN_ASSERT(registry.all_of<Component>(entity));
        insert_components<Component>(view, op, entity);
    }

    template<typename Component, typename It>
    void remove(const entt::registry &registry, It first, It last, bool check = false) {
        insert_components<Component>(registry, registry_op_type::remove, first, last, check);
    }

    template<typename Component>
    void remove(const entt::registry &registry) {
        auto view = registry.view<Component>();
        remove<Component>(registry, view.begin(), view.end());
    }

    template<typename Component>
    void remove(const entt::registry &registry, entt::entity entity) {
        auto &op = find_or_create_component_operation<Component>(registry_op_type::remove);
        auto view = registry.view<Component>();
        EDYN_ASSERT(!registry.all_of<Component>(entity));
        insert_components<Component>(view, op, entity);
    }

    virtual void emplace_all(const entt::registry &registry, const std::vector<entt::entity> &entities) = 0;
    virtual void replace_all(const entt::registry &registry, const std::vector<entt::entity> &entities) = 0;
    virtual void remove_all(const entt::registry &registry, const std::vector<entt::entity> &entities) = 0;

    virtual void emplace_all(const entt::registry &registry, entt::entity entity) = 0;
    virtual void replace_all(const entt::registry &registry, entt::entity entity) = 0;
    virtual void remove_all(const entt::registry &registry, entt::entity entity) = 0;

    virtual void emplace_type_id(const entt::registry &registry, entt::entity entity, entt::id_type id) = 0;
    virtual void replace_type_id(const entt::registry &registry, entt::entity entity, entt::id_type id) = 0;
    virtual void remove_type_id(const entt::registry &registry, entt::entity entity, entt::id_type id) = 0;

    void add_entity_mapping(entt::entity local_entity, entt::entity remote_entity) {
        auto &op = find_or_create_component_operation<entt::entity>(registry_op_type::ent_map);
        op.entities.push_back(local_entity);
        auto *components = static_cast<component_operation_impl<entt::entity> *>(op.components.get());
        components->components.push_back(remote_entity);
    }

    registry_operation_collection finish() {
        return registry_operation_collection{std::move(operations)};
    }

private:
    std::vector<registry_operation> operations;
};

template<typename... Components>
class registry_operations_builder_impl : public registry_operations_builder {
public:
    void emplace_all(const entt::registry &registry, const std::vector<entt::entity> &entities) override {
        (emplace<Components>(registry, entities.begin(), entities.end(), true), ...);
    }

    void replace_all(const entt::registry &registry, const std::vector<entt::entity> &entities) override {
        (replace<Components>(registry, entities.begin(), entities.end(), true), ...);
    }

    void remove_all(const entt::registry &registry, const std::vector<entt::entity> &entities) override {
        (remove<Components>(registry, entities.begin(), entities.end(), true), ...);
    }

    void emplace_all(const entt::registry &registry, entt::entity entity) override {
        ((registry.all_of<Components>(entity) ? emplace<Components>(registry, entity) : (void)0), ...);
    }

    void replace_all(const entt::registry &registry, entt::entity entity) override {
        ((registry.all_of<Components>(entity) ? replace<Components>(registry, entity) : (void)0), ...);
    }

    void remove_all(const entt::registry &registry, entt::entity entity) override {
        ((registry.all_of<Components>(entity) ? remove<Components>(registry, entity) : (void)0), ...);
    }

    void emplace_type_id(const entt::registry &registry, entt::entity entity, entt::id_type id) override {
        ((entt::type_seq<Components>::value() == id ? emplace<Components>(registry, entity) : (void)0), ...);
    }

    void replace_type_id(const entt::registry &registry, entt::entity entity, entt::id_type id) override {
        ((entt::type_seq<Components>::value() == id ? replace<Components>(registry, entity) : (void)0), ...);
    }

    void remove_type_id(const entt::registry &registry, entt::entity entity, entt::id_type id) override {
        ((entt::type_seq<Components>::value() == id ? remove<Components>(registry, entity) : (void)0), ...);
    }
};

}

#endif // EDYN_UTIL_REGISTRY_OPERATION_HPP
