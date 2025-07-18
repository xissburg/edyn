#ifndef EDYN_NETWORKING_EXTRAPOLATION_REQUEST_HPP
#define EDYN_NETWORKING_EXTRAPOLATION_REQUEST_HPP

#include "edyn/networking/extrapolation/extrapolation_context.hpp"
#include "edyn/networking/packet/registry_snapshot.hpp"
#include "edyn/parallel/message_queue.hpp"
#include <entt/entity/sparse_set.hpp>

namespace edyn {

struct extrapolation_request {
    message_queue_identifier destination;
    double start_time;
    packet::registry_snapshot snapshot;
    extrapolation_context context;
    double execution_time_limit {0.4};
    bool should_remap {true};
};

}

#endif // EDYN_NETWORKING_EXTRAPOLATION_REQUEST_HPP
