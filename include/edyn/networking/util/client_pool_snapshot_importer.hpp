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
    void import_components(entt::registry &registry, const entity_map &emap,
                           const std::vector<entt::entity> &entities,
                           const std::vector<Component> &components,
                           bool mark_dirty) {
        EDYN_ASSERT(entities.size() == components.size());
        for (size_t i = 0; i < entities.size(); ++i) {
            auto remote_entity = entities[i];

            if (!emap.contains(remote_entity)) {
                continue;
            }

            auto local_entity = emap.at(remote_entity);

            if (!registry.valid(local_entity)) {
                continue;
            }

            auto comp = components[i];
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
    void import_components_empty(entt::registry &registry, const entity_map &emap,
                                 const std::vector<entt::entity> &entities, bool mark_dirty) {
        for (auto remote_entity : entities) {
            if (!emap.contains(remote_entity)) {
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
    void import_components_local(entt::registry &registry,
                                 const std::vector<entt::entity> &entities,
                                 const std::vector<Component> &components,
                                 bool mark_dirty) {
        EDYN_ASSERT(entities.size() == components.size());
        for (auto ite = entities.begin(), itc = components.begin();
             ite != entities.end() && itc != components.end(); ++ite, ++itc) {
            auto local_entity = *ite;

            if (!registry.valid(local_entity)) {
                continue;
            }

            auto &comp = *itc;

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
    void import_components_empty_local(entt::registry &registry,
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
            auto data = std::static_pointer_cast<pool_snapshot_data_impl<Component>>(pool.ptr);

            if constexpr(std::is_empty_v<Component>) {
                import_components_empty<Component>(registry, emap, data->entities, mark_dirty);
            } else {
                import_components<Component>(registry, emap, data->entities, data->components, mark_dirty);
            }
        });
    }

    void import_local(entt::registry &registry, const pool_snapshot &pool, bool mark_dirty) override {
        auto all_components = std::tuple<Components...>{};

        visit_tuple(all_components, pool.component_index, [&] (auto &&c) {
            using Component = std::decay_t<decltype(c)>;
            auto data = std::static_pointer_cast<pool_snapshot_data_impl<Component>>(pool.ptr);

            if constexpr(std::is_empty_v<Component>) {
                import_components_empty_local<Component>(registry, data->entities, mark_dirty);
            } else {
                import_components_local<Component>(registry, data->entities, data->components, mark_dirty);
            }
        });
    }
};

}

#endif // EDYN_NETWORKING_UTIL_CLIENT_POOL_SNAPSHOT_IMPORTER_HPP
