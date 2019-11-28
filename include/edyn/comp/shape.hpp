#ifndef EDYN_COMP_SHAPE_HPP
#define EDYN_COMP_SHAPE_HPP

#include <variant>
#include "edyn/shapes/plane_shape.hpp"
#include "edyn/shapes/sphere_shape.hpp"

namespace edyn {

struct shape {
    std::variant<plane_shape, sphere_shape> var;
};

}

#endif // EDYN_COMP_SHAPE_HPP