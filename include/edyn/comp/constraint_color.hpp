#ifndef EDYN_COMP_CONSTRAINT_COLOR_HPP
#define EDYN_COMP_CONSTRAINT_COLOR_HPP

#include <limits>

namespace edyn {

struct constraint_color {
    static constexpr size_t none = std::numeric_limits<size_t>::max();
    size_t value {none};
};

}

#endif // EDYN_COMP_CONSTRAINT_COLOR_HPP