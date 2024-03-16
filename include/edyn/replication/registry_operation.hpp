#ifndef EDYN_REPLICATION_REGISTRY_OPERATION_HPP
#define EDYN_REPLICATION_REGISTRY_OPERATION_HPP

#include <entt/core/type_info.hpp>
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

enum class registry_operation_type {
    create,
    destroy,
    emplace,
    replace,
    remove,
    map_entity
};

struct operation_base {
    entt::entity entity;

    virtual ~operation_base() = default;

    virtual void execute(entt::registry &registry, entity_map &entity_map) const = 0;
    virtual void execute(entt::registry &registry) const = 0;

    virtual void remap(const entity_map &emap) = 0;

    virtual entt::id_type payload_type_id() const = 0;
    virtual registry_operation_type operation_type() const = 0;

    template<typename... Ts>
    bool payload_type_any_of() const {
        const auto id = payload_type_id();
        return ((entt::type_index<Ts>::value() == id) || ...);
    }

    template<typename... Ts>
    bool payload_type_any_of([[maybe_unused]] const std::tuple<Ts...> &) const {
        return payload_type_any_of<Ts...>();
    }
};

struct operation_create : public operation_base {
    void execute(entt::registry &registry, entity_map &entity_map) const override {
         if (!entity_map.contains(entity)) {
            auto local_entity = registry.create();
            entity_map.insert(entity, local_entity);
         }
    }

    void execute(entt::registry &registry) const override {}

    void remap(const entity_map &emap) override {
        entity = emap.at(entity);
    }

    entt::id_type payload_type_id() const override {
        return entt::type_index<void>::value();
    }

    registry_operation_type operation_type() const override {
        return registry_operation_type::create;
    }
};

struct operation_destroy : public operation_base {
    void execute(entt::registry &registry, entity_map &entity_map) const override {
        if (!entity_map.contains(entity)) return;

        auto local_entity = entity_map.at(entity);
        entity_map.erase(entity);

        if (registry.valid(local_entity)) {
            registry.destroy(local_entity);
        }
    }

    void execute(entt::registry &registry) const override {
        if (registry.valid(entity)) {
            registry.destroy(entity);
        }
    }

    void remap(const entity_map &emap) override {
        entity = emap.at(entity);
    }

    entt::id_type payload_type_id() const override {
        return entt::type_index<void>::value();
    }

    registry_operation_type operation_type() const override {
        return registry_operation_type::destroy;
    }
};

struct operation_map_entity : public operation_base {
    entt::entity local_entity;

    void execute(entt::registry &registry, entity_map &entity_map) const override {
        if (registry.valid(local_entity)) {
            if (entity_map.contains(entity)) {
                EDYN_ASSERT(entity_map.at(entity) == local_entity);
            } else {
                entity_map.insert(entity, local_entity);
            }
        }
    }

    void execute(entt::registry &registry) const override {}

    void remap(const entity_map &emap) override {
        entity = emap.at(entity);
        local_entity = emap.at_local(local_entity);
    }

    entt::id_type payload_type_id() const override {
        return entt::type_index<entt::entity>::value();
    }

    registry_operation_type operation_type() const override {
        return registry_operation_type::map_entity;
    }
};

template<typename Component>
struct operation_emplace : public operation_base {
    Component component;
    static constexpr auto is_empty_type = std::is_empty_v<Component>;

    void execute(entt::registry &registry, entity_map &entity_map) const override {
        if (!entity_map.contains(entity)) {
            return;
        }

        auto local_entity = entity_map.at(entity);

        if (!registry.valid(local_entity) || registry.all_of<Component>(local_entity)) {
            return;
        }

        if constexpr(is_empty_type) {
            registry.emplace<Component>(local_entity);
        } else {
            auto comp = component;
            internal::map_child_entity(registry, entity_map, comp);
            registry.emplace<Component>(local_entity, comp);
        }
    }

    void execute(entt::registry &registry) const override {
        if (!registry.valid(entity) || registry.all_of<Component>(entity)) {
            return;
        }

        if constexpr(is_empty_type) {
            registry.emplace<Component>(entity);
        } else {
            registry.emplace<Component>(entity, component);
        }
    }

    void remap(const entity_map &emap) override {
        entity = emap.at(entity);
        internal::map_child_entity_no_validation(emap, component);
    }

    entt::id_type payload_type_id() const override {
        return entt::type_index<Component>::value();
    }

    registry_operation_type operation_type() const override {
        return registry_operation_type::emplace;
    }
};

template<typename Component>
struct operation_replace : public operation_base {
    Component component;
    static constexpr auto is_empty_type = std::is_empty_v<Component>;

    void execute(entt::registry &registry, entity_map &entity_map) const override {
        if constexpr(!std::is_empty_v<Component>) {
            if (!entity_map.contains(entity)) {
                return;
            }

            auto local_entity = entity_map.at(entity);

            if (!registry.valid(local_entity) || !registry.all_of<Component>(local_entity)) {
                return;
            }

            auto comp = component;
            internal::map_child_entity(registry, entity_map, comp);
            registry.patch<Component>(local_entity, [&comp](auto &&current) {
                merge_component(current, comp);
            });
        }
    }

    void execute(entt::registry &registry) const override {
        if constexpr(!std::is_empty_v<Component>) {
            if (!registry.valid(entity) || !registry.all_of<Component>(entity)) {
                return;
            }

            registry.patch<Component>(entity, [&](auto &&current) {
                merge_component(current, component);
            });
        }
    }

    void remap(const entity_map &emap) override {
        entity = emap.at(entity);
        internal::map_child_entity_no_validation(emap, component);
    }

    entt::id_type payload_type_id() const override {
        return entt::type_index<Component>::value();
    }

    registry_operation_type operation_type() const override {
        return registry_operation_type::replace;
    }
};

template<typename Component>
struct operation_remove : public operation_base {
    void execute(entt::registry &registry, entity_map &entity_map) const override {
        if (!entity_map.contains(entity)) {
            return;
        }

        auto local_entity = entity_map.at(entity);

        if (!registry.valid(local_entity)) {
            return;
        }

        registry.remove<Component>(local_entity);
    }

    void execute(entt::registry &registry) const override {
        if (registry.valid(entity)) {
            registry.remove<Component>(entity);
        }
    }

    void remap(const entity_map &emap) override {
        entity = emap.at(entity);
    }

    entt::id_type payload_type_id() const override {
        return entt::type_index<Component>::value();
    }

    registry_operation_type operation_type() const override {
        return registry_operation_type::remove;
    }
};

/**
 * @brief An operation to replicate contents of one registry into another.
 * It contains an array of buffers which hold the data of each operation.
 * Placement new is used to allocate operation objects into these buffers.
 */
class registry_operation final {
public:
    static constexpr auto default_block_size{128ul};
    std::vector<std::vector<uint8_t>> data_blocks;
    std::vector<operation_base *> operations;

    registry_operation() {
        auto data = std::vector<uint8_t>{};
        data.resize(default_block_size);
        data_blocks.emplace_back(std::move(data));
    }

    registry_operation(registry_operation &&other) {
        data_blocks = std::move(other.data_blocks);
        operations = std::move(other.operations);

        // Ensure there's always one data block.
        auto data = std::vector<uint8_t>{};
        data.resize(default_block_size);
        other.data_blocks.emplace_back(std::move(data));
    }

    registry_operation & operator=(registry_operation &&other) {
        data_blocks = std::move(other.data_blocks);
        operations = std::move(other.operations);

        auto data = std::vector<uint8_t>{};
        data.resize(default_block_size);
        other.data_blocks.emplace_back(std::move(data));

        return *this;
    }

    registry_operation(registry_operation &) = delete;
    registry_operation & operator=(registry_operation &) = delete;

    ~registry_operation() {
        for (auto *op : operations) {
            op->~operation_base();
        }
    }

    template<typename... Func>
    void execute(entt::registry &registry, entity_map &entity_map, Func... func) const {
        for (auto *op : operations) {
            op->execute(registry, entity_map);
            (func(op), ...);
        }
    }

    void execute(entt::registry &registry) const {
        for (auto *op : operations) {
            op->execute(registry);
        }
    }

    void remap(const entity_map &emap) {
        for (auto *op : operations) {
            op->remap(emap);
        }
    }

    bool empty() const {
        return operations.empty();
    }
};

}

#endif // EDYN_REPLICATION_REGISTRY_OPERATION_HPP
