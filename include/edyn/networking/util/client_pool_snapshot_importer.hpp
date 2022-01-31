#ifndef EDYN_NETWORKING_UTIL_CLIENT_POOL_SNAPSHOT_IMPORTER_HPP
#define EDYN_NETWORKING_UTIL_CLIENT_POOL_SNAPSHOT_IMPORTER_HPP

#include <entt/entity/registry.hpp>
#include "edyn/collision/contact_manifold.hpp"
#include "edyn/comp/dirty.hpp"
#include "edyn/networking/sys/client_side.hpp"
#include "edyn/parallel/island_delta_builder.hpp"
#include "edyn/parallel/merge/merge_component.hpp"
#include "edyn/networking/packet/util/pool_snapshot.hpp"
#include "edyn/edyn.hpp"

namespace edyn {

bool client_owns_entity(entt::registry &registry, entt::entity entity);

class client_pool_snapshot_importer {
public:
    virtual void import(entt::registry &, entity_map &, const pool_snapshot &,
                        std::vector<entt::entity> &unknown_entities) = 0;
    virtual void insert_remote_non_procedural_to_builder(
                                        const entt::registry &registry,
                                        const std::vector<pool_snapshot> &pools,
                                        const entity_map &emap,
                                        island_delta_builder &builder) = 0;
    virtual void insert_non_procedural_to_builder(const entt::registry &, entt::entity,
                                                  island_delta_builder &) = 0;
};

template<typename... Components>
class client_pool_snapshot_importer_impl : public client_pool_snapshot_importer {

    template<typename Component>
    void import_pairs(entt::registry &registry, entity_map &emap,
                      const std::vector<std::pair<entt::entity, Component>> &pairs,
                      std::vector<entt::entity> &unknown_entities) {
        for (auto &pair : pairs) {
            auto remote_entity = pair.first;

            if (!emap.has_rem(remote_entity)) {
                unknown_entities.push_back(remote_entity);
                continue;
            }

            auto local_entity = emap.remloc(remote_entity);

            if (!registry.valid(local_entity)) {
                emap.erase_loc(local_entity);
                continue;
            }

            if constexpr(std::is_empty_v<Component>) {
                if (!registry.any_of<Component>(local_entity)) {
                    registry.emplace<Component>(local_entity);
                    registry.emplace_or_replace<dirty>(local_entity).template created<Component>();
                }
            } else {
                auto comp = pair.second;
                merge(comp, emap);

                if (registry.any_of<Component>(local_entity)) {
                    registry.replace<Component>(local_entity, comp);
                    refresh<Component>(registry, local_entity);
                } else {
                    registry.emplace<Component>(local_entity, comp);
                    registry.emplace_or_replace<dirty>(local_entity).template created<Component>();
                }
            }
        }
    }

    template<typename Component>
    static void insert_remote_to_builder(
                const entt::registry &registry,
                const std::vector<std::pair<entt::entity, Component>> &pairs,
                entity_map &emap, island_delta_builder &builder) {

        for (auto &pair : pairs) {
            auto remote_entity = pair.first;

            if (!emap.has_rem(pair.first)) {
                continue;
            }

            auto local_entity = emap.remloc(remote_entity);

            if (!client_owns_entity(registry, local_entity)) {
                builder.updated(local_entity, pair.second);
            }
        }
    }

    template<typename Component>
    static void insert_to_builder(const entt::registry &registry, entt::entity entity, island_delta_builder &builder) {
        if (auto *comp = registry.try_get<Component>(entity)) {
            builder.updated(entity, *comp);
        }
    }

public:
    using insert_remote_np_to_builder_func_t =
        void(const entt::registry &, const std::vector<pool_snapshot> &pools,
             const entity_map &emap, island_delta_builder &builder);
    insert_remote_np_to_builder_func_t *insert_remote_np_to_builder_func;

    using insert_to_builder_func_t = void(const entt::registry &registry,
                                          entt::entity entity,
                                          island_delta_builder &builder);
    insert_to_builder_func_t *insert_to_builder_func;

    template<typename... NonProcedural>
    client_pool_snapshot_importer_impl([[maybe_unused]] std::tuple<Components...>,
                                       [[maybe_unused]] std::tuple<NonProcedural...>) {
        static_assert((!std::is_empty_v<NonProcedural> && ...));

        insert_remote_np_to_builder_func = [] (const entt::registry &registry, const std::vector<pool_snapshot> &pools,
                                               const entity_map &emap, island_delta_builder &builder) {
            auto all_components = std::tuple<Components...>{};

            for (auto &pool : pools) {
                visit_tuple(all_components, pool.component_index, [&] (auto &&c) {
                    using Component = std::decay_t<decltype(c)>;
                    if constexpr(has_type<Component, std::tuple<NonProcedural...>>::value) {
                        auto &pairs = std::static_pointer_cast<pool_snapshot_data<Component>>(pool.ptr)->pairs;
                        insert_remote_to_builder(registry, pairs, emap, builder);
                    }
                });
            }
        };

        insert_to_builder_func = [] (const entt::registry &registry, entt::entity entity, island_delta_builder &builder) {
            (insert_to_builder<NonProcedural>(registry, entity, builder), ...);
        };
    }

    void import(entt::registry &registry, entity_map &emap, const pool_snapshot &pool,
                std::vector<entt::entity> &unknown_entities) override {
        auto all_components = std::tuple<Components...>{};

        visit_tuple(all_components, pool.component_index, [&] (auto &&c) {
            using Component = std::decay_t<decltype(c)>;
            auto &pairs = std::static_pointer_cast<pool_snapshot_data<Component>>(pool.ptr)->pairs;
            import_pairs(registry, emap, pairs, unknown_entities);
        });
    }

    void insert_remote_non_procedural_to_builder(
                                        const entt::registry &registry,
                                        const std::vector<pool_snapshot> &pools,
                                        const entity_map &emap,
                                        island_delta_builder &builder) override {
        insert_remote_np_to_builder_func(registry, pools, emap, builder);
    }

    void insert_non_procedural_to_builder(const entt::registry &registry, entt::entity entity,
                                          island_delta_builder &builder) override {
        insert_to_builder_func(registry, entity, builder);
    }
};

}

#endif // EDYN_NETWORKING_UTIL_CLIENT_POOL_SNAPSHOT_IMPORTER_HPP
