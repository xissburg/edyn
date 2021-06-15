#ifndef EDYN_COMP_COLLISION_FILTER_HPP
#define EDYN_COMP_COLLISION_FILTER_HPP

#include <cstdint>

namespace edyn {

struct collision_filter {
    uint64_t group {~0ULL};
    uint64_t mask {~0ULL};
};

}

#endif // EDYN_COMP_COLLISION_FILTER_HPP
