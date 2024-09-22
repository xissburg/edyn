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
    static constexpr auto data_block_unit_size {32ul};
    static constexpr auto max_block_size {8192ul};

    template<typename T, typename... Args>
    T * make_op(Args &&... args) {
        constexpr unsigned long size = sizeof(T);
        static_assert(size <= max_block_size, "Component size larger than maximum block size.");

        // Create new data block if current block size would be exceeded.
        if (m_data_index + size > operation.data_blocks.back().size()) {
            auto data = std::vector<uint8_t>{};
            data.resize(std::min(size * data_block_unit_size, max_block_size));
            operation.data_blocks.emplace_back(std::move(data));
            m_data_index = 0;
        }

        auto buff = &operation.data_blocks.back()[m_data_index];
        m_data_index += size;

        // Use placement new to allocate object in the current buffer.
        auto *op = new(buff) T(std::forward<Args>(args)...);
        operation.operations.push_back(op);

        return op;
    }

public:
    registry_operation_builder(entt::registry &registry) : registry(&registry) {}
    virtual ~registry_operation_builder() = default;

    template<typename It>
    void create(It first, It last) {
        for (; first != last; ++first) {
            auto *op = make_op<operation_create>();
            op->entity = *first;
        }
    }

    void create(entt::entity entity) {
        auto *op = make_op<operation_create>();
        op->entity = entity;
    }

    template<typename It>
    void destroy(It first, It last) {
        for (; first != last; ++first) {
            auto *op = make_op<operation_destroy>();
            op->entity = *first;
        }
    }

    void destroy(entt::entity entity) {
        auto *op = make_op<operation_destroy>();
        op->entity = entity;
    }

    template<typename Component, typename It>
    void emplace(It first, It last) {
        auto view = registry->view<Component>();

        for (; first != last; ++first) {
            auto *op = make_op<operation_emplace<Component>>();
            op->entity = *first;

            if constexpr(!std::is_empty_v<Component>) {
                op->component = view.template get<Component>(op->entity);
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
        auto *op = make_op<operation_emplace<Component>>();
        op->entity = entity;

        if constexpr(!std::is_empty_v<Component>) {
            op->component = registry->get<Component>(entity);
        }
    }

    template<typename Component, typename It>
    void replace(It first, It last) {
        auto view = registry->view<Component>();

        for (; first != last; ++first) {
            auto *op = make_op<operation_replace<Component>>();
            op->entity = *first;

            if constexpr(!std::is_empty_v<Component>) {
                op->component = view.template get<Component>(op->entity);
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
        auto *op = make_op<operation_replace<Component>>();
        op->entity = entity;

        if constexpr(!std::is_empty_v<Component>) {
            op->component = registry->get<Component>(entity);
        }
    }

    template<typename Component>
    void replace(entt::entity entity, const Component &comp) {
        auto *op = make_op<operation_replace<Component>>();
        op->entity = entity;
        op->component = comp;
    }

    template<typename Component, typename It>
    void remove(It first, It last) {
        for (; first != last; ++first) {
            auto *op = make_op<operation_remove<Component>>();
            op->entity = *first;
        }
    }

    template<typename Component>
    void remove() {
        auto view = registry->view<Component>();
        remove<Component>(view.begin(), view.end());
    }

    template<typename Component>
    void remove(entt::entity entity) {
        auto *op = make_op<operation_remove<Component>>();
        op->entity = entity;
    }

    void add_entity_mapping(entt::entity local_entity, entt::entity remote_entity) {
        auto *op = make_op<operation_map_entity>();
        op->entity = local_entity;
        op->local_entity = remote_entity;
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

protected:
    entt::registry *registry;
    registry_operation operation;
    size_t m_data_index {};
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
        (emplace<Components>(entities.begin(), entities.end()), ...);
    }

    void replace_all(const std::vector<entt::entity> &entities) override {
        (replace<Components>(entities.begin(), entities.end()), ...);
    }

    void remove_all(const std::vector<entt::entity> &entities) override {
        (remove<Components>(entities.begin(), entities.end()), ...);
    }

    void emplace_all(const entt::sparse_set &entities) override {
        (emplace<Components>(entities.begin(), entities.end()), ...);
    }

    void replace_all(const entt::sparse_set &entities) override {
        (replace<Components>(entities.begin(), entities.end()), ...);
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
