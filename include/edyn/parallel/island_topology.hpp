#ifndef EDYN_PARALLEL_ISLAND_TOPOLOGY_HPP
#define EDYN_PARALLEL_ISLAND_TOPOLOGY_HPP

#include <vector>
#include <cstdint>

namespace edyn {

struct island_topology {
    std::vector<size_t> component_sizes;
};

}

#endif // EDYN_PARALLEL_ISLAND_TOPOLOGY_HPP