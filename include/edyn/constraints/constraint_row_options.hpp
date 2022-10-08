#ifndef EDYN_CONSTRAINTS_CONSTRAINT_ROW_OPTIONS_HPP
#define EDYN_CONSTRAINTS_CONSTRAINT_ROW_OPTIONS_HPP

#include "edyn/math/scalar.hpp"

namespace edyn {

/**
 * Optional info to be used when setting up a constraint row.
 */
struct constraint_row_options {
    scalar error {scalar(0)};

    // Error reduction parameter.
    scalar erp {scalar(0.2)};

    scalar restitution {scalar(0)};
};

}

#endif // EDYN_CONSTRAINTS_CONSTRAINT_ROW_OPTIONS_HPP
