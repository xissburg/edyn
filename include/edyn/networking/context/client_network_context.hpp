#ifndef EDYN_NETWORKING_CLIENT_NETWORKING_CONTEXT_HPP
#define EDYN_NETWORKING_CLIENT_NETWORKING_CONTEXT_HPP

#include "edyn/util/entity_map.hpp"
#include "edyn/networking/util/comp_state_history.hpp"
#include "edyn/networking/util/client_pool_snapshot_importer.hpp"
#include "edyn/networking/util/client_pool_snapshot_exporter.hpp"
#include "edyn/networking/util/clock_sync.hpp"
#include <entt/entity/fwd.hpp>
#include <entt/entity/entity.hpp>
#include <entt/entity/sparse_set.hpp>
#include <entt/signal/sigh.hpp>
#include <cstdint>
#include <memory>

namespace edyn {

namespace packet {
    struct edyn_packet;
}

class extrapolation_job;
struct extrapolation_component_pool;

struct extrapolation_job_context {
    std::unique_ptr<extrapolation_job> job;
};

struct client_network_context {
    client_network_context();

    entt::entity client_entity {entt::null};
    entt::sparse_set owned_entities {};

    edyn::entity_map entity_map;
    std::vector<entt::entity> created_entities;
    std::vector<entt::entity> destroyed_entities;
    bool importing_entities {false};

    // Set of entities requested via `packet::entity_request`. Note that these
    // are entities in the server's registry space.
    entt::sparse_set requested_entities {};

    double last_snapshot_time {0};
    double server_playout_delay {0.3};

    std::vector<extrapolation_job_context> extrapolation_jobs;
    std::shared_ptr<comp_state_history> state_history;

    using request_entity_func_t = void(entt::entity);
    entt::sigh<request_entity_func_t> request_entity_signal;

    auto request_entity_sink() {
        return entt::sink{request_entity_signal};
    }

    using packet_observer_func_t = void(const packet::edyn_packet &);
    entt::sigh<packet_observer_func_t> packet_signal;

    auto packet_sink() {
        return entt::sink{packet_signal};
    }

    entt::sigh<void(void)> client_entity_assigned_signal;
    auto client_entity_assigned_sink() {
        return entt::sink{client_entity_assigned_signal};
    }

    entt::sigh<void(void)> extrapolation_timeout_signal;
    auto extrapolation_timeout_sink() {
        return entt::sink{extrapolation_timeout_signal};
    }

    std::shared_ptr<client_pool_snapshot_importer> pool_snapshot_importer;
    std::shared_ptr<client_pool_snapshot_exporter> pool_snapshot_exporter;

    using extrapolation_component_pool_import_func_t =
        void(std::vector<std::shared_ptr<extrapolation_component_pool>> &,
             const entt::registry &, const entt::sparse_set &entities);
    extrapolation_component_pool_import_func_t *extrapolation_component_pool_import_func;

    using extrapolation_component_pool_import_by_id_func_t =
        void(std::vector<std::shared_ptr<extrapolation_component_pool>> &,
             const entt::registry &, const entt::sparse_set &entities, entt::id_type);
    extrapolation_component_pool_import_by_id_func_t *extrapolation_component_pool_import_by_id_func;

    using is_input_component_func_t =
        bool(entt::id_type);
    is_input_component_func_t *is_input_component_func;

    clock_sync_data clock_sync;
};

}

#endif // EDYN_NETWORKING_CLIENT_NETWORKING_CONTEXT_HPP
