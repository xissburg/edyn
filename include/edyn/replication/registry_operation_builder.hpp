#ifndef EDYN_REPLICATION_REGISTRY_OPERATION_BUILDER_HPP
#define EDYN_REPLICATION_REGISTRY_OPERATION_BUILDER_HPP

#include <vector>
#include <memory>
#include <entt/entity/registry.hpp>
#include "edyn/replication/registry_operation.hpp"

namespace edyn {

/**
 * @brief Utility to build a registry operation collection bit by bit.
 */
class registry_operation_builder {
    template<typename Component>
    registry_operation & find_or_create_component_operation(registry_op_type op_type) {
        EDYN_ASSERT(op_type == registry_op_type::emplace ||
                    op_type == registry_op_type::replace ||
                    op_type == registry_op_type::remove ||
                    op_type == registry_op_type::ent_map);

        auto type_id = entt::type_index<Component>::value();

        for (auto &op : operations) {
            if (op.operation == op_type && op.components && op.components->get_type_id() == type_id) {
                return op;
            }
        }

        auto &op = operations.emplace_back();
        op.operation = op_type;
        op.components = std::make_unique<component_operation_impl<Component>>();
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
    void insert_components(registry_op_type op_type, It first, It last, bool check = false) {
        if (first == last) {
            return;
        }

        EDYN_ASSERT(op_type == registry_op_type::emplace ||
                    op_type == registry_op_type::replace ||
                    op_type == registry_op_type::remove);

        auto &op = find_or_create_component_operation<Component>(op_type);
        auto view = registry->view<Component>();

        if (!check || op_type == registry_op_type::remove) {
            for (; first != last; ++first) {
                auto entity = *first;
                insert_components<Component>(view, op, entity);
            }
        } else {
            for (; first != last; ++first) {
                auto entity = *first;

                if (view.contains(entity)) {
                    insert_components<Component>(view, op, entity);
                }
            }
        }
    }

public:
    registry_operation_builder(entt::registry &registry) : registry(&registry) {}
    virtual ~registry_operation_builder() = default;

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
    void emplace(It first, It last, bool check = false) {
        insert_components<Component>(registry_op_type::emplace, first, last, check);
    }

    template<typename Component>
    void emplace() {
        auto view = registry->view<Component>();
        emplace<Component>(view.begin(), view.end());
    }

    template<typename Component>
    void emplace(entt::entity entity) {
        auto &op = find_or_create_component_operation<Component>(registry_op_type::emplace);
        auto view = registry->view<Component>();
        EDYN_ASSERT(registry->all_of<Component>(entity));
        insert_components<Component>(view, op, entity);
    }

    template<typename Component, typename It>
    void replace(It first, It last, bool check = false) {
        insert_components<Component>(registry_op_type::replace, first, last, check);
    }

    template<typename Component>
    void replace() {
        auto view = registry->view<Component>();
        replace<Component>(view.begin(), view.end());
    }

    template<typename Component>
    void replace(entt::entity entity) {
        auto &op = find_or_create_component_operation<Component>(registry_op_type::replace);
        auto view = registry->view<Component>();
        EDYN_ASSERT(registry->all_of<Component>(entity));
        insert_components<Component>(view, op, entity);
    }

    template<typename Component>
    void replace(entt::entity entity, const Component &comp) {
        auto &op = find_or_create_component_operation<Component>(registry_op_type::replace);
        op.entities.push_back(entity);
        auto *components = static_cast<component_operation_impl<Component> *>(op.components.get());
        components->components.push_back(comp);
    }

    template<typename Component, typename It>
    void remove(It first, It last, bool check = false) {
        insert_components<Component>(registry_op_type::remove, first, last, check);
    }

    template<typename Component>
    void remove() {
        auto view = registry->view<Component>();
        remove<Component>(view.begin(), view.end());
    }

    template<typename Component>
    void remove(entt::entity entity) {
        auto &op = find_or_create_component_operation<Component>(registry_op_type::remove);
        auto view = registry->view<Component>();
        insert_components<Component>(view, op, entity);
    }

    virtual void emplace_all(const std::vector<entt::entity> &entities) = 0;
    virtual void replace_all(const std::vector<entt::entity> &entities) = 0;
    virtual void remove_all(const std::vector<entt::entity> &entities) = 0;

    virtual void emplace_all(const entt::sparse_set &entities) = 0;
    virtual void replace_all(const entt::sparse_set &entities) = 0;
    virtual void remove_all(const entt::sparse_set &entities) = 0;

    virtual void emplace_all(entt::entity entity) = 0;
    virtual void replace_all(entt::entity entity) = 0;
    virtual void remove_all(entt::entity entity) = 0;

    virtual void emplace_type_id(entt::entity entity, entt::id_type id) = 0;
    virtual void replace_type_id(entt::entity entity, entt::id_type id) = 0;
    virtual void remove_type_id(entt::entity entity, entt::id_type id) = 0;

    template<typename It>
    void emplace_type_ids(entt::entity entity, It first, It last) {
        for (; first != last; ++first) {
            emplace_type_id(entity, *first);
        }
    }

    template<typename It>
    void replace_type_ids(entt::entity entity, It first, It last) {
        for (; first != last; ++first) {
            replace_type_id(entity, *first);
        }
    }

    template<typename It>
    void remove_type_ids(entt::entity entity, It first, It last) {
        for (; first != last; ++first) {
            remove_type_id(entity, *first);
        }
    }

    void add_entity_mapping(entt::entity local_entity, entt::entity remote_entity) {
        auto &op = find_or_create_component_operation<entt::entity>(registry_op_type::ent_map);
        op.entities.push_back(local_entity);
        auto *components = static_cast<component_operation_impl<entt::entity> *>(op.components.get());
        components->components.push_back(remote_entity);
    }

    bool empty() const {
        for (auto &op : operations) {
            if (!op.entities.empty()) {
                return false;
            }
        }

        return true;
    }

    registry_operation_collection finish() {
        return registry_operation_collection{std::move(operations)};
    }

    entt::registry & get_registry() {
        return *registry;
    }

protected:
    entt::registry *registry;
    std::vector<registry_operation> operations;
};

template<typename... Components>
class registry_operation_builder_impl : public registry_operation_builder {
public:
    registry_operation_builder_impl() = default;
    registry_operation_builder_impl(entt::registry &registry, [[maybe_unused]] std::tuple<Components...>)
        : registry_operation_builder(registry) {}

    void emplace_all(const std::vector<entt::entity> &entities) override {
        (emplace<Components>(entities.begin(), entities.end(), true), ...);
    }

    void replace_all(const std::vector<entt::entity> &entities) override {
        (replace<Components>(entities.begin(), entities.end(), true), ...);
    }

    void remove_all(const std::vector<entt::entity> &entities) override {
        (remove<Components>(entities.begin(), entities.end(), true), ...);
    }

    void emplace_all(const entt::sparse_set &entities) override {
        (emplace<Components>(entities.begin(), entities.end(), true), ...);
    }

    void replace_all(const entt::sparse_set &entities) override {
        (replace<Components>(entities.begin(), entities.end(), true), ...);
    }

    void remove_all(const entt::sparse_set &entities) override {
        (remove<Components>(entities.begin(), entities.end(), true), ...);
    }

    void emplace_all(entt::entity entity) override {
        ((registry->all_of<Components>(entity) ? emplace<Components>(entity) : (void)0), ...);
    }

    void replace_all(entt::entity entity) override {
        ((registry->all_of<Components>(entity) ? replace<Components>(entity) : (void)0), ...);
    }

    void remove_all(entt::entity entity) override {
        ((registry->all_of<Components>(entity) ? remove<Components>(entity) : (void)0), ...);
    }

    void emplace_type_id(entt::entity entity, entt::id_type id) override {
        ((entt::type_index<Components>::value() == id ? emplace<Components>(entity) : (void)0), ...);
    }

    void replace_type_id(entt::entity entity, entt::id_type id) override {
        ((entt::type_index<Components>::value() == id ? replace<Components>(entity) : (void)0), ...);
    }

    void remove_type_id(entt::entity entity, entt::id_type id) override {
        ((entt::type_index<Components>::value() == id ? remove<Components>(entity) : (void)0), ...);
    }
};

}

#endif // EDYN_REPLICATION_REGISTRY_OPERATION_BUILDER_HPP
