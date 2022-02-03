#ifndef EDYN_NETWORKING_EXTRAPOLATION_INPUT_HPP
#define EDYN_NETWORKING_EXTRAPOLATION_INPUT_HPP

#include "edyn/util/entity_map.hpp"
#include "edyn/networking/extrapolation_component_pool.hpp"
#include "edyn/networking/packet/transient_snapshot.hpp"
#include <entt/entity/fwd.hpp>
#include <entt/entity/sparse_set.hpp>

namespace edyn {

struct extrapolation_input {
    double start_time;
    entt::sparse_set entities;
    std::vector<std::unique_ptr<extrapolation_component_pool>> pools;
    packet::transient_snapshot transient_snapshot;

    using extrapolation_component_pool_import_by_id_func_t =
        void(std::vector<std::unique_ptr<extrapolation_component_pool>> &,
             const entt::registry &, const entt::sparse_set &entities, entt::id_type);
    extrapolation_component_pool_import_by_id_func_t *extrapolation_component_pool_import_by_id_func {nullptr};
};

}

#endif // EDYN_NETWORKING_EXTRAPOLATION_INPUT_HPP
