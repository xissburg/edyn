#ifndef EDYN_PARALLEL_MESSAGE_HPP
#define EDYN_PARALLEL_MESSAGE_HPP

#include <vector>
#include <cstdint>
#include "edyn/comp.hpp"
#include "edyn/parallel/registry_snapshot.hpp"

namespace edyn::msg {

struct registry_snapshot {
    decltype(edyn::registry_snapshot(all_components{})) snapshot;
};

struct set_paused {
    bool paused;
};

struct step_simulation {

};

}

#endif // EDYN_PARALLEL_MESSAGE_HPP