#ifndef EDYN_PARALLEL_MESSAGE_HPP
#define EDYN_PARALLEL_MESSAGE_HPP

#include "edyn/math/scalar.hpp"
#include "edyn/context/settings.hpp"
#include "edyn/dynamics/material_mixing.hpp"

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

}

#endif // EDYN_PARALLEL_MESSAGE_HPP
