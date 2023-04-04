#ifndef EDYN_REPLICATION_REGISTRY_OPERATION_BUILDER_HPP
#define EDYN_REPLICATION_REGISTRY_OPERATION_BUILDER_HPP

#include <type_traits>
#include <vector>
#include <memory>
#include <entt/entity/registry.hpp>
#include "edyn/replication/registry_operation.hpp"
#include "edyn/util/vector_util.hpp"

namespace edyn {

/**
 * @brief Utility to build a registry operation collection bit by bit.
 */
class registry_operation_builder {

    template<typename Component>
    component_operation_emplace<Component> & find_or_create_emplace_operation() {
         auto found_it = std::find_if(operation.emplace_components.begin(), operation.emplace_components.end(),
                                      [](auto &&op) { return op->get_type_id() == entt::type_index<Component>::value(); });
        if (found_it != operation.emplace_components.end()) {
            return static_cast<component_operation_emplace<Component> &>(*found_it->get());
        }

        auto op = std::make_unique<component_operation_emplace<Component>>();
        auto &ref = *op.get();
        operation.emplace_components.emplace_back(std::move(op));
        return ref;
    }

    template<typename Component>
    component_operation_replace<Component> & find_or_create_replace_operation() {
         auto found_it = std::find_if(operation.replace_components.begin(), operation.replace_components.end(),
                                     [](auto &&op) { return op->get_type_id() == entt::type_index<Component>::value(); });
        if (found_it != operation.replace_components.end()) {
            return static_cast<component_operation_replace<Component> &>(*found_it->get());
        }

        auto op = std::make_unique<component_operation_replace<Component>>();
        auto &ref = *op.get();
        operation.replace_components.emplace_back(std::move(op));
        return ref;
    }

    template<typename Component>
    component_operation_remove<Component> & find_or_create_remove_operation() {
         auto found_it = std::find_if(operation.remove_components.begin(), operation.remove_components.end(),
                                     [](auto &&op) { return op->get_type_id() == entt::type_index<Component>::value(); });
        if (found_it != operation.remove_components.end()) {
            return static_cast<component_operation_remove<Component> &>(*found_it->get());
        }

        auto op = std::make_unique<component_operation_remove<Component>>();
        auto &ref = *op.get();
        operation.remove_components.emplace_back(std::move(op));
        return ref;
    }

public:
    registry_operation_builder(entt::registry &registry) : registry(&registry) {}
    virtual ~registry_operation_builder() = default;

    template<typename It>
    void create(It first, It last) {
        operation.create_entities.insert(operation.create_entities.end(), first, last);
    }

    void create(entt::entity entity) {
        operation.create_entities.push_back(entity);
    }

    template<typename It>
    void destroy(It first, It last) {
        operation.destroy_entities.insert(operation.destroy_entities.end(), first, last);
    }

    void destroy(entt::entity entity) {
        operation.destroy_entities.push_back(entity);
    }

    template<typename Component, typename It>
    void emplace(It first, It last, bool check = false) {
        if (first == last) {
            return;
        }

        auto view = registry->view<Component>();

        if (!check) {
            auto &op = find_or_create_emplace_operation<Component>();
            op.entities.insert(op.entities.end(), first, last);

            if constexpr(!std::is_empty_v<Component>) {
                for (; first != last; ++first) {
                    auto entity = *first;
                    auto [comp] = view.get(entity);
                    op.components.push_back(comp);
                }
            }
        } else {
            // Make sure there is at least one entity which has the component
            // to avoid creating an empty operation.
            bool contains = false;

            for (; first != last; ++first) {
                auto entity = *first;

                if (view.contains(entity)) {
                    contains = true;
                    break;
                }
            }

            if (!contains) {
                return;
            }

            auto &op = find_or_create_emplace_operation<Component>();

            for (; first != last; ++first) {
                auto entity = *first;

                if (view.contains(entity)) {
                    op.entities.push_back(entity);

                    if constexpr(!std::is_empty_v<Component>) {
                        auto [comp] = view.get(entity);
                        op.components.push_back(comp);
                    }
                }
            }
        }
    }

    template<typename Component>
    void emplace() {
        auto view = registry->view<Component>();
        emplace<Component>(view.begin(), view.end());
    }

    template<typename Component>
    void emplace(entt::entity entity) {
        auto &op = find_or_create_emplace_operation<Component>();
        op.entities.push_back(entity);

        if constexpr(!std::is_empty_v<Component>) {
            auto &comp = registry->get<Component>(entity);
            op.components.push_back(comp);
        }
    }

    template<typename Component, typename It>
    void replace(It first, It last, bool check = false) {
        if (first == last) {
            return;
        }

        auto &op = find_or_create_replace_operation<Component>();
        auto view = registry->view<Component>();

        if (!check) {
            if constexpr(!std::is_empty_v<Component>) {
                for (; first != last; ++first) {
                    auto entity = *first;
                    auto [comp] = view.get(entity);
                    bool found = false;

                    for (size_t i = 0; i < op.entities.size(); ++i) {
                        if (op.entities[i] == entity) {
                            op.components[i] = comp;
                            found = true;
                            break;
                        }
                    }

                    if (!found) {
                        op.entities.push_back(entity);
                        op.components.push_back(comp);
                    }
                }
            } else {
                for (; first != last; ++first) {
                    auto entity = *first;
                    if (!vector_contains(op.entities, entity)) {
                        op.entities.push_back(entity);
                        break;
                    }
                }
            }
        } else {
            for (; first != last; ++first) {
                auto entity = *first;

                if (view.contains(entity)) {
                    size_t idx = SIZE_MAX;

                    for (size_t i = 0; i < op.entities.size(); ++i) {
                        if (op.entities[i] == entity) {
                            idx = i;
                            break;
                        }
                    }

                    if (idx == SIZE_MAX) {
                        op.entities.push_back(entity);

                        if constexpr(!std::is_empty_v<Component>) {
                            op.components.push_back(std::get<0>(view.get(entity)));
                        }
                    } else {
                        if constexpr(!std::is_empty_v<Component>) {
                            op.components[idx] = std::get<0>(view.get(entity));
                        }
                    }
                }
            }
        }
    }

    template<typename Component>
    void replace() {
        auto view = registry->view<Component>();
        replace<Component>(view.begin(), view.end());
    }

    template<typename Component>
    void replace(entt::entity entity) {
        auto &op = find_or_create_replace_operation<Component>();
        size_t idx = SIZE_MAX;

        for (size_t i = 0; i < op.entities.size(); ++i) {
            if (op.entities[i] == entity) {
                idx = i;
                break;
            }
        }

        if (idx == SIZE_MAX) {
            op.entities.push_back(entity);

            if constexpr(!std::is_empty_v<Component>) {
                op.components.push_back(registry->get<Component>(entity));
            }
        } else {
            if constexpr(!std::is_empty_v<Component>) {
                op.components[idx] = registry->get<Component>(entity);
            }
        }
    }

    template<typename Component>
    void replace(entt::entity entity, const Component &comp) {
        auto &op = find_or_create_replace_operation<Component>();
        size_t idx = SIZE_MAX;

        for (size_t i = 0; i < op.entities.size(); ++i) {
            if (op.entities[i] == entity) {
                idx = i;
                break;
            }
        }

        if (idx == SIZE_MAX) {
            op.entities.push_back(entity);
            op.components.push_back(comp);
        } else {
            op.components[idx] = comp;
        }
    }

    template<typename Component, typename It>
    void remove(It first, It last) {
        if (first == last) {
            return;
        }

        auto &op = find_or_create_remove_operation<Component>();
        op.entities.insert(op.entities.end(), first, last);
    }

    template<typename Component>
    void remove() {
        auto view = registry->view<Component>();
        remove<Component>(view.begin(), view.end());
    }

    template<typename Component>
    void remove(entt::entity entity) {
        auto &op = find_or_create_remove_operation<Component>();
        op.entities.push_back(entity);
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
        operation.map_entities.emplace_back(local_entity, remote_entity);
    }

    bool empty() const {
        return operation.empty();
    }

    registry_operation && finish() {
        return std::move(operation);
    }

    entt::registry & get_registry() {
        return *registry;
    }

protected:
    entt::registry *registry;
    registry_operation operation;
};

template<typename... Components>
class registry_operation_builder_impl : public registry_operation_builder {
public:
    registry_operation_builder_impl() = default;

    registry_operation_builder_impl(entt::registry &registry)
        : registry_operation_builder(registry) {}

    registry_operation_builder_impl(entt::registry &registry, [[maybe_unused]] std::tuple<Components...>)
        : registry_operation_builder(registry) {}

    void emplace_all(const std::vector<entt::entity> &entities) override {
        (emplace<Components>(entities.begin(), entities.end(), true), ...);
    }

    void replace_all(const std::vector<entt::entity> &entities) override {
        (replace<Components>(entities.begin(), entities.end(), true), ...);
    }

    void remove_all(const std::vector<entt::entity> &entities) override {
        (remove<Components>(entities.begin(), entities.end()), ...);
    }

    void emplace_all(const entt::sparse_set &entities) override {
        (emplace<Components>(entities.begin(), entities.end(), true), ...);
    }

    void replace_all(const entt::sparse_set &entities) override {
        (replace<Components>(entities.begin(), entities.end(), true), ...);
    }

    void remove_all(const entt::sparse_set &entities) override {
        (remove<Components>(entities.begin(), entities.end()), ...);
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
