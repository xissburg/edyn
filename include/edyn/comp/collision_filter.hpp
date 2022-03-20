#ifndef EDYN_COMP_COLLISION_FILTER_HPP
#define EDYN_COMP_COLLISION_FILTER_HPP

#include <cstdint>

namespace edyn {

struct collision_filter {
    uint64_t group {~0ULL};
    uint64_t mask {~0ULL};
};

template<typename Archive>
void serialize(Archive &archive, collision_filter &c) {
    archive(c.group);
    archive(c.mask);
}

}

#endif // EDYN_COMP_COLLISION_FILTER_HPP
