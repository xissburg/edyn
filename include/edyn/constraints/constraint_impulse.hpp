#ifndef EDYN_CONSTRAINTS_CONSTRAINT_IMPULSE_HPP
#define EDYN_CONSTRAINTS_CONSTRAINT_IMPULSE_HPP

#include <array>
#include "edyn/config/constants.hpp"

namespace edyn {

struct constraint_impulse {
    std::array<scalar, max_constraint_rows> values;

    void zero_out() {
        for (auto &imp : values) {
            imp = 0;
        }
    }
};

}

#endif // EDYN_CONSTRAINTS_CONSTRAINT_IMPULSE_HPP
