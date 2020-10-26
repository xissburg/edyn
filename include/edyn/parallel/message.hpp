#ifndef EDYN_PARALLEL_MESSAGE_HPP
#define EDYN_PARALLEL_MESSAGE_HPP

#include <vector>
#include <entt/fwd.hpp>

namespace edyn::msg {

struct set_paused {
    bool paused;
};

struct step_simulation {

};

struct split_island {
    std::vector<std::vector<entt::entity>> connected_components;
};

}

#endif // EDYN_PARALLEL_MESSAGE_HPP