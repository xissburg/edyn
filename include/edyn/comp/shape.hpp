#ifndef EDYN_COMP_SHAPE_HPP
#define EDYN_COMP_SHAPE_HPP

#include <variant>
#include "edyn/shapes/sphere_shape.hpp"

namespace edyn {

struct shape {
    std::variant<sphere_shape> var;
};

}

#endif // EDYN_COMP_SHAPE_HPP