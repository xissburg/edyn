#ifndef EDYN_NETWORKING_SERVER_NETWORK_CONTEXT_HPP
#define EDYN_NETWORKING_SERVER_NETWORK_CONTEXT_HPP

#include <vector>
#include <entt/entity/fwd.hpp>
#include "edyn/networking/packet/util/pool_snapshot.hpp"
#include "edyn/networking/util/server_pool_snapshot_importer.hpp"
#include "edyn/networking/util/server_pool_snapshot_exporter.hpp"

namespace edyn {

struct server_network_context {
    server_network_context();

    std::vector<entt::entity> pending_created_clients;

    std::shared_ptr<server_pool_snapshot_importer> pool_snapshot_importer;
    std::shared_ptr<server_pool_snapshot_exporter> pool_snapshot_exporter;
};

}

#endif // EDYN_NETWORKING_SERVER_NETWORK_CONTEXT_HPP
