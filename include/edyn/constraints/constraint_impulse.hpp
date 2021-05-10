#ifndef EDYN_CONSTRAINTS_CONSTRAINT_IMPULSE_HPP
#define EDYN_CONSTRAINTS_CONSTRAINT_IMPULSE_HPP

#include <array>
#include "edyn/config/constants.hpp"

namespace edyn {

/**
 * @brief Holds the impulses that have been applied to a constraint in the last
 * solver update. This information needs to be stored to be used for warm starting
 * the next update. It is kept in a separate component because this needs to be
 * continuously sent back to the main registry so that when an island merge/split
 * occurs, the latest constraint row impulses can be migrated into the other
 * island and the simulation can continue with this information included.
 */
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
