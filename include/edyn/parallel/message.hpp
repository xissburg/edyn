#ifndef EDYN_PARALLEL_MESSAGE_HPP
#define EDYN_PARALLEL_MESSAGE_HPP

#include <entt/entity/fwd.hpp>
#include "edyn/math/vector3.hpp"
#include "edyn/context/settings.hpp"
#include "edyn/dynamics/material_mixing.hpp"
#include "edyn/networking/util/pool_snapshot.hpp"
#include "edyn/util/registry_operation.hpp"

namespace edyn::msg {

struct set_paused {
    bool paused;
};

struct set_settings {
    edyn::settings settings;
};

struct set_material_table {
    edyn::material_mix_table table;
};

struct step_simulation {};

struct wake_up_island {};

struct split_island {};

struct set_com {
    entt::entity entity;
    vector3 com;
};

struct apply_network_pools {
    std::vector<entt::entity> entities;
    std::vector<pool_snapshot> pools;
};

struct island_reg_ops {
    registry_operation_collection ops;
};

}

#endif // EDYN_PARALLEL_MESSAGE_HPP
