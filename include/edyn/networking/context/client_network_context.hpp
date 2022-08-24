#ifndef EDYN_NETWORKING_CLIENT_NETWORKING_CONTEXT_HPP
#define EDYN_NETWORKING_CLIENT_NETWORKING_CONTEXT_HPP

#include "edyn/replication/entity_map.hpp"
#include "edyn/networking/util/input_state_history.hpp"
#include "edyn/networking/util/client_snapshot_importer.hpp"
#include "edyn/networking/util/client_snapshot_exporter.hpp"
#include "edyn/networking/util/clock_sync.hpp"
#include "edyn/networking/extrapolation/extrapolation_job.hpp"
#include "edyn/networking/extrapolation/extrapolation_modified_comp.hpp"
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

struct extrapolation_job_context {
    std::unique_ptr<extrapolation_job> job;

    extrapolation_job_context() = default;
    extrapolation_job_context(std::unique_ptr<extrapolation_job> &&job)
        : job(std::move(job))
    {}
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

    // Without full ownership, the client will only send input components to server.
    bool allow_full_ownership {true};

    std::vector<extrapolation_job_context> extrapolation_jobs;
    std::shared_ptr<input_state_history> input_history;

    using packet_observer_func_t = void(const packet::edyn_packet &);
    entt::sigh<packet_observer_func_t> packet_signal;

    auto packet_sink() {
        return entt::sink{packet_signal};
    }

    entt::sigh<void(entt::entity)> client_assigned_signal;
    auto client_assigned_sink() {
        return entt::sink{client_assigned_signal};
    }

    entt::sigh<void(void)> extrapolation_timeout_signal;
    auto extrapolation_timeout_sink() {
        return entt::sink{extrapolation_timeout_signal};
    }

    make_extrapolation_modified_comp_func_t *make_extrapolation_modified_comp;

    std::shared_ptr<client_snapshot_importer> snapshot_importer;
    std::shared_ptr<client_snapshot_exporter> snapshot_exporter;

    clock_sync_data clock_sync;
};

}

#endif // EDYN_NETWORKING_CLIENT_NETWORKING_CONTEXT_HPP
