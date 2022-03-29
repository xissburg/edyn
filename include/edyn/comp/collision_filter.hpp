#ifndef EDYN_COMP_COLLISION_FILTER_HPP
#define EDYN_COMP_COLLISION_FILTER_HPP

#include <cstdint>

namespace edyn {

struct collision_filter {
    static constexpr uint64_t all_groups = ~0ull;
    uint64_t group {all_groups};
    uint64_t mask {all_groups};
};

template<typename Archive>
void serialize(Archive &archive, collision_filter &c) {
    archive(c.group);
    archive(c.mask);
}

}

#endif // EDYN_COMP_COLLISION_FILTER_HPP
