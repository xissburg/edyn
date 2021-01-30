#ifndef EDYN_COMP_CONSTRAINT_GROUP_HPP
#define EDYN_COMP_CONSTRAINT_GROUP_HPP

#include <cstddef>

namespace edyn {

struct constraint_group {
    static constexpr size_t stitch_group = std::numeric_limits<size_t>::max();
    size_t value;
};

}

#endif // EDYN_COMP_CONSTRAINT_GROUP_HPP