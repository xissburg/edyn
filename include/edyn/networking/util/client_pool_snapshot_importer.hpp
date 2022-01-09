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
    virtual void import(entt::registry &registry, const pool_snapshot &pool) = 0;
};

template<typename... Components>
class client_pool_snapshot_importer_impl : public client_pool_snapshot_importer {

    template<typename Component>
    void import_pairs(entt::registry &registry, const std::vector<std::pair<entt::entity, Component>> &pairs) {
        auto &ctx = registry.ctx<client_networking_context>();
        auto merge_ctx = merge_context{&registry, &ctx.entity_map};

        for (auto &pair : pairs) {
            auto remote_entity = pair.first;

            if (!ctx.entity_map.has_rem(pair.first)) {
                // Entity not present in client. Send an entity request to server.
                ctx.request_entity_signal.publish(remote_entity);
                continue;
            }

            auto local_entity = ctx.entity_map.remloc(remote_entity);

            if (!registry.valid(local_entity)) {
                ctx.entity_map.erase_loc(local_entity);
                continue;
            }

            if constexpr(std::is_empty_v<Component>) {
                if (!registry.any_of<Component>(local_entity)) {
                    registry.emplace<Component>(local_entity);
                    registry.emplace_or_replace<dirty>(local_entity).template created<Component>();
                }
            } else {
                auto comp = pair.second;
                merge(static_cast<Component *>(nullptr), comp, merge_ctx);

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

    void import(entt::registry &registry, const pool_snapshot &pool) override {
        auto all_components = std::tuple<Components...>{};

        visit_tuple(all_components, pool.component_index, [&] (auto &&c) {
            using Component = std::decay_t<decltype(c)>;
            import_pairs(registry, std::static_pointer_cast<pool_snapshot_data<Component>>(pool.ptr)->pairs);
        });
    }
};

}

#endif // EDYN_NETWORKING_UTIL_CLIENT_POOL_SNAPSHOT_IMPORTER_HPP
