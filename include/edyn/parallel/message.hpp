#ifndef EDYN_PARALLEL_MESSAGE_HPP
#define EDYN_PARALLEL_MESSAGE_HPP

#include <vector>
#include <unordered_set>
#include <entt/fwd.hpp>

namespace edyn::msg {

struct set_paused {
    bool paused;
};

struct step_simulation {};

struct wake_up_island {};

}

#endif // EDYN_PARALLEL_MESSAGE_HPP