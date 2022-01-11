#ifndef EDYN_NETWORKING_UTIL_CLIENT_POOL_SNAPSHOT_IMPORTER_HPP
#define EDYN_NETWORKING_UTIL_CLIENT_POOL_SNAPSHOT_IMPORTER_HPP

#include <entt/entity/registry.hpp>
#include "edyn/collision/contact_manifold.hpp"
#include "edyn/comp/dirty.hpp"
#include "edyn/parallel/merge/merge_component.hpp"
#include "edyn/networking/context/client_networking_context.hpp"
#include "edyn/networking/packet/util/pool_snapshot.hpp"
#include "edyn/edyn.hpp"

namespace edyn {

class client_pool_snapshot_importer {
public:
    virtual void import(entt::registry &, entity_map &, const pool_snapshot &) = 0;
};

template<typename... Components>
class client_pool_snapshot_importer_impl : public client_pool_snapshot_importer {

    template<typename Component>
    void import_pairs(entt::registry &registry, entity_map &emap, const std::vector<std::pair<entt::entity, Component>> &pairs) {
        auto *client_ctx = registry.try_ctx<client_networking_context>();

        for (auto &pair : pairs) {
            auto remote_entity = pair.first;

            if (!emap.has_rem(pair.first)) {
                if (client_ctx) {
                    // Entity not present in client. Send an entity request to server.
                    client_ctx->request_entity_signal.publish(remote_entity);
                }
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

public:
    client_pool_snapshot_importer_impl([[maybe_unused]] std::tuple<Components...>) {}

    void import(entt::registry &registry, entity_map &emap, const pool_snapshot &pool) override {
        auto all_components = std::tuple<Components...>{};

        visit_tuple(all_components, pool.component_index, [&] (auto &&c) {
            using Component = std::decay_t<decltype(c)>;
            import_pairs(registry, emap, std::static_pointer_cast<pool_snapshot_data<Component>>(pool.ptr)->pairs);
        });
    }
};

}

#endif // EDYN_NETWORKING_UTIL_CLIENT_POOL_SNAPSHOT_IMPORTER_HPP
