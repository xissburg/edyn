#ifndef EDYN_NETWORKING_SERVER_NETWORKING_CONTEXT_HPP
#define EDYN_NETWORKING_SERVER_NETWORKING_CONTEXT_HPP

#include <vector>
#include <entt/entity/fwd.hpp>
#include "edyn/networking/packet/util/pool_snapshot.hpp"
#include "edyn/networking/util/networked_component_index_source.hpp"

namespace edyn {

struct pool_snapshot;
void import_pool_server_default(entt::registry &, entt::entity client_entity, const pool_snapshot &, bool broadcast);
void insert_entity_components_default(entt::registry &, entt::entity entity,
                                      std::vector<pool_snapshot> &pools);
void insert_transient_components_default(entt::registry &, entt::entity entity,
                                         std::vector<pool_snapshot> &pools);

struct server_networking_context {
    std::vector<entt::entity> pending_created_clients;
    std::shared_ptr<networked_component_index_source> index_source {new networked_component_index_source_impl(networked_components)};

    using import_pool_func_t = decltype(&import_pool_server_default);
    import_pool_func_t import_pool_func {&import_pool_server_default};

    using insert_entity_components_func_t = decltype(&insert_entity_components_default);
    insert_entity_components_func_t insert_entity_components_func {&insert_entity_components_default};

    using insert_transient_components_func_t = decltype(&insert_transient_components_default);
    insert_transient_components_func_t insert_transient_components_func {&insert_transient_components_default};
};

}

#endif // EDYN_NETWORKING_SERVER_NETWORKING_CONTEXT_HPP
