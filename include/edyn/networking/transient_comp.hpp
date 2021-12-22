#ifndef EDYN_NETWORKING_TRANSIENT_COMP_HPP
#define EDYN_NETWORKING_TRANSIENT_COMP_HPP

#include "edyn/comp/angvel.hpp"
#include "edyn/comp/linvel.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/comp/position.hpp"
#include <tuple>

namespace edyn {

static const auto transient_components = std::tuple<
    position,
    orientation,
    linvel,
    angvel
>{};

}

#endif // EDYN_NETWORKING_TRANSIENT_COMP_HPP
