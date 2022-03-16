#ifndef EDYN_NETWORKING_UTIL_CLIENT_POOL_SNAPSHOT_IMPORTER_HPP
#define EDYN_NETWORKING_UTIL_CLIENT_POOL_SNAPSHOT_IMPORTER_HPP

#include <entt/entity/registry.hpp>
#include "edyn/networking/comp/network_dirty.hpp"
#include "edyn/networking/sys/client_side.hpp"
#include "edyn/networking/packet/util/pool_snapshot.hpp"
#include "edyn/edyn.hpp"

namespace edyn {

bool client_owns_entity(const entt::registry &registry, entt::entity entity);

class client_pool_snapshot_importer {
public:
    virtual ~client_pool_snapshot_importer() = default;
    virtual void import(entt::registry &, const entity_map &, const pool_snapshot &, bool mark_dirty) = 0;
    virtual void import_local(entt::registry &, const pool_snapshot &, bool mark_dirty) = 0;
};

template<typename... Components>
class client_pool_snapshot_importer_impl : public client_pool_snapshot_importer {

    template<typename Component>
    void import_pairs(entt::registry &registry, const entity_map &emap,
                      const std::vector<std::pair<entt::entity, Component>> &pairs,
                      bool mark_dirty) {
        for (auto &pair : pairs) {
            auto remote_entity = pair.first;

            if (!emap.count(remote_entity)) {
                continue;
            }

            auto local_entity = emap.at(remote_entity);

            if (!registry.valid(local_entity)) {
                continue;
            }

            auto comp = pair.second;
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

    template<typename Component>
    void import_entities(entt::registry &registry, const entity_map &emap,
                         const std::vector<entt::entity> &entities, bool mark_dirty) {
        for (auto remote_entity : entities) {
            if (!emap.count(remote_entity)) {
                continue;
            }

            auto local_entity = emap.at(remote_entity);

            if (!registry.valid(local_entity)) {
                continue;
            }

            if (!registry.any_of<Component>(local_entity)) {
                registry.emplace<Component>(local_entity);

                if (mark_dirty) {
                    registry.get_or_emplace<network_dirty>(local_entity).template created<Component>();
                }
            }
        }
    }

    template<typename Component>
    void import_pairs_local(entt::registry &registry,
                            const std::vector<std::pair<entt::entity, Component>> &pairs,
                            bool mark_dirty) {
        for (auto &pair : pairs) {
            auto local_entity = pair.first;

            if (!registry.valid(local_entity)) {
                continue;
            }

            auto &comp = pair.second;

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

    template<typename Component>
    void import_entities_local(entt::registry &registry,
                               const std::vector<entt::entity> &entities,
                               bool mark_dirty) {
        for (auto local_entity : entities) {
            if (!registry.valid(local_entity)) {
                continue;
            }

            if (!registry.any_of<Component>(local_entity)) {
                registry.emplace<Component>(local_entity);

                if (mark_dirty) {
                    registry.get_or_emplace<network_dirty>(local_entity).template created<Component>();
                }
            }
        }
    }

public:
    client_pool_snapshot_importer_impl([[maybe_unused]] std::tuple<Components...>) {}

    void import(entt::registry &registry, const entity_map &emap, const pool_snapshot &pool, bool mark_dirty) override {
        auto all_components = std::tuple<Components...>{};

        visit_tuple(all_components, pool.component_index, [&] (auto &&c) {
            using Component = std::decay_t<decltype(c)>;
            auto &data = std::static_pointer_cast<pool_snapshot_data_impl<Component>>(pool.ptr)->data;

            if constexpr(std::is_empty_v<Component>) {
                import_entities<Component>(registry, emap, data, mark_dirty);
            } else {
                import_pairs<Component>(registry, emap, data, mark_dirty);
            }
        });
    }

    void import_local(entt::registry &registry, const pool_snapshot &pool, bool mark_dirty) override {
        auto all_components = std::tuple<Components...>{};

        visit_tuple(all_components, pool.component_index, [&] (auto &&c) {
            using Component = std::decay_t<decltype(c)>;
            auto &data = std::static_pointer_cast<pool_snapshot_data_impl<Component>>(pool.ptr)->data;

            if constexpr(std::is_empty_v<Component>) {
                import_entities_local<Component>(registry, data, mark_dirty);
            } else {
                import_pairs_local<Component>(registry, data, mark_dirty);
            }
        });
    }
};

}

#endif // EDYN_NETWORKING_UTIL_CLIENT_POOL_SNAPSHOT_IMPORTER_HPP
