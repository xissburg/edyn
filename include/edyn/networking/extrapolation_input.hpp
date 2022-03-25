#ifndef EDYN_NETWORKING_EXTRAPOLATION_INPUT_HPP
#define EDYN_NETWORKING_EXTRAPOLATION_INPUT_HPP

#include "edyn/util/entity_map.hpp"
#include "edyn/networking/packet/transient_snapshot.hpp"
#include "edyn/util/registry_operation.hpp"
#include <entt/entity/fwd.hpp>
#include <entt/entity/sparse_set.hpp>
#include <memory>

namespace edyn {

struct extrapolation_input {
    double start_time;
    entt::sparse_set entities {};
    entt::sparse_set owned_entities {};
    registry_operation_collection ops;
    packet::transient_snapshot transient_snapshot;
    double execution_time_limit {0.4};
    bool should_remap {true};

    using is_input_component_func_t = bool(entt::id_type);
    is_input_component_func_t *is_input_component_func;
};

}

#endif // EDYN_NETWORKING_EXTRAPOLATION_INPUT_HPP
