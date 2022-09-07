#ifndef EDYN_NETWORKING_EXTRAPOLATION_INPUT_HPP
#define EDYN_NETWORKING_EXTRAPOLATION_INPUT_HPP

#include "edyn/replication/entity_map.hpp"
#include "edyn/networking/packet/registry_snapshot.hpp"
#include "edyn/replication/registry_operation.hpp"
#include <entt/entity/fwd.hpp>
#include <entt/entity/sparse_set.hpp>
#include <memory>

namespace edyn {

struct extrapolation_input {
    double start_time;
    entt::sparse_set entities {};
    entt::sparse_set owned_entities {};
    registry_operation ops;
    packet::registry_snapshot snapshot;
    double execution_time_limit {0.4};
    bool should_remap {true};
};

}

#endif // EDYN_NETWORKING_EXTRAPOLATION_INPUT_HPP
