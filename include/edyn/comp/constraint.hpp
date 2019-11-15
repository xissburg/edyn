#ifndef EDYN_COMP_CONSTRAINT_HPP
#define EDYN_COMP_CONSTRAINT_HPP

#include <variant>
#include "edyn/constraints/distance.hpp"

namespace edyn {

struct constraint {
    std::variant<distance_constraint> var;
};

}

#endif // EDYN_COMP_CONSTRAINT_HPP