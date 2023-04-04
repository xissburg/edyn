#ifndef EDYN_NETWORKING_EXTRAPOLATION_REQUEST_HPP
#define EDYN_NETWORKING_EXTRAPOLATION_REQUEST_HPP

#include "edyn/parallel/message_queue.hpp"
#include "edyn/networking/packet/registry_snapshot.hpp"
#include "edyn/replication/registry_operation.hpp"
#include <entt/entity/sparse_set.hpp>

namespace edyn {

struct extrapolation_request {
    message_queue_identifier destination;
    double start_time;
    entt::sparse_set entities {};
    entt::sparse_set owned_entities {};
    registry_operation ops;
    packet::registry_snapshot snapshot;
    double execution_time_limit {0.4};
    bool should_remap {true};
};

}

#endif // EDYN_NETWORKING_EXTRAPOLATION_REQUEST_HPP
