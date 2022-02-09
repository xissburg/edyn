#ifndef EDYN_NETWORKING_EXTRAPOLATION_INPUT_HPP
#define EDYN_NETWORKING_EXTRAPOLATION_INPUT_HPP

#include "edyn/util/entity_map.hpp"
#include "edyn/networking/extrapolation_component_pool.hpp"
#include "edyn/networking/packet/transient_snapshot.hpp"
#include <entt/entity/fwd.hpp>
#include <entt/entity/sparse_set.hpp>
#include <memory>

namespace edyn {

struct extrapolation_input {
    double start_time;
    entt::sparse_set entities;
    entt::sparse_set owned_entities;
    std::vector<std::shared_ptr<extrapolation_component_pool>> pools;
    packet::transient_snapshot transient_snapshot;
    double execution_time_limit {0.4};

    using extrapolation_component_pool_import_by_id_func_t =
        void(std::vector<std::shared_ptr<extrapolation_component_pool>> &,
             const entt::registry &, const entt::sparse_set &entities, entt::id_type);
    extrapolation_component_pool_import_by_id_func_t *extrapolation_component_pool_import_by_id_func {nullptr};

    using is_non_procedural_component_func_t = bool(entt::id_type);
    is_non_procedural_component_func_t *is_non_procedural_component_func;
};

}

#endif // EDYN_NETWORKING_EXTRAPOLATION_INPUT_HPP
