#ifndef EDYN_NETWORKING_UTIL_CLIENT_SNAPSHOT_IMPORTER_HPP
#define EDYN_NETWORKING_UTIL_CLIENT_SNAPSHOT_IMPORTER_HPP

#include <entt/entity/registry.hpp>
#include "edyn/networking/comp/network_dirty.hpp"
#include "edyn/networking/sys/client_side.hpp"
#include "edyn/networking/util/registry_snapshot.hpp"
#include "edyn/edyn.hpp"

namespace edyn {

class client_snapshot_importer {
public:
    virtual ~client_snapshot_importer() = default;

    // Load remote snapshot into a registry.
    virtual void import(entt::registry &registry, const entity_map &emap,
                        const registry_snapshot &snap, bool mark_dirty) = 0;

    // Load a snapshot that's been mapped to local (using
    // `registry_snapshot::convert_remloc`) into a registry.
    virtual void import_local(entt::registry &registry,
                              const registry_snapshot &snap, bool mark_dirty) = 0;
};

template<typename... Components>
class client_snapshot_importer_impl : public client_snapshot_importer {

    template<typename Component>
    void import_components(entt::registry &registry, const entity_map &emap,
                           const std::vector<entt::entity> &entities,
                           const pool_snapshot_data_impl<Component> &pool,
                           bool mark_dirty) {
        for (size_t i = 0; i < pool.entity_indices.size(); ++i) {
            auto entity_index = pool.entity_indices[i];
            auto remote_entity = entities[entity_index];

            if (!emap.contains(remote_entity)) {
                continue;
            }

            auto local_entity = emap.at(remote_entity);

            if (!registry.valid(local_entity)) {
                continue;
            }

            if constexpr(std::is_empty_v<Component>) {
                if (!registry.any_of<Component>(local_entity)) {
                    registry.emplace<Component>(local_entity);

                    if (mark_dirty) {
                        registry.get_or_emplace<network_dirty>(local_entity).template created<Component>();
                    }
                }
            } else {
                auto comp = pool.components[i];
                internal::map_child_entity(registry, emap, comp);

                if (mark_dirty) {
                    // Mark as dirty using `network_dirty` to avoid having these
                    // components being sent back to the server later on in
                    // `client_side` when dirty components are put into a
                    // `general_snapshot` and dispatched to the server.
                    auto &dirty = registry.get_or_emplace<network_dirty>(local_entity);

                    if (registry.any_of<Component>(local_entity)) {
                        dirty.template updated<Component>();
                    } else {
                        dirty.template created<Component>();
                    }
                }

                if (registry.any_of<Component>(local_entity)) {
                    registry.replace<Component>(local_entity, comp);
                } else {
                    registry.emplace<Component>(local_entity, comp);
                }
            }
        }
    }

    template<typename Component>
    void import_components_local(entt::registry &registry,
                                 const std::vector<entt::entity> &entities,
                                 const pool_snapshot_data_impl<Component> &pool,
                                 bool mark_dirty) {
        for (size_t i = 0; i < pool.entity_indices.size(); ++i) {
            auto entity_index = pool.entity_indices[i];
            auto local_entity = entities[entity_index];

            if (!registry.valid(local_entity)) {
                continue;
            }

            if constexpr(std::is_empty_v<Component>) {
                if (!registry.any_of<Component>(local_entity)) {
                    registry.emplace<Component>(local_entity);

                    if (mark_dirty) {
                        registry.get_or_emplace<network_dirty>(local_entity).template created<Component>();
                    }
                }
            } else {
                auto &comp = pool.components[i];

                if (mark_dirty) {
                    auto &dirty = registry.get_or_emplace<network_dirty>(local_entity);

                    if (registry.any_of<Component>(local_entity)) {
                        dirty.template updated<Component>();
                    } else {
                        dirty.template created<Component>();
                    }
                }

                if (registry.any_of<Component>(local_entity)) {
                    registry.replace<Component>(local_entity, comp);
                } else {
                    registry.emplace<Component>(local_entity, comp);
                }
            }
        }
    }

public:
    client_snapshot_importer_impl([[maybe_unused]] std::tuple<Components...>) {}

    void import(entt::registry &registry, const entity_map &emap,
                const registry_snapshot &snap, bool mark_dirty) override {
        auto all_components = std::tuple<Components...>{};

        for (auto &pool : snap.pools) {
            visit_tuple(all_components, pool.component_index, [&] (auto &&c) {
                using Component = std::decay_t<decltype(c)>;
                auto *typed_pool = static_cast<pool_snapshot_data_impl<Component> *>(pool.ptr.get());
                import_components(registry, emap, snap.entities, *typed_pool, mark_dirty);
            });
        }
    }

    void import_local(entt::registry &registry,
                      const registry_snapshot &snap, bool mark_dirty) override {
        auto all_components = std::tuple<Components...>{};

        for (auto &pool : snap.pools) {
            visit_tuple(all_components, pool.component_index, [&] (auto &&c) {
                using Component = std::decay_t<decltype(c)>;
                auto *typed_pool = static_cast<pool_snapshot_data_impl<Component> *>(pool.ptr.get());
                import_components_local<Component>(registry, snap.entities, *typed_pool, mark_dirty);
            });
        }
    }
};

}

#endif // EDYN_NETWORKING_UTIL_CLIENT_SNAPSHOT_IMPORTER_HPP
