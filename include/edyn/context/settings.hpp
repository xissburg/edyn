#ifndef EDYN_CONTEXT_SETTINGS_HPP
#define EDYN_CONTEXT_SETTINGS_HPP

#include "edyn/math/scalar.hpp"

namespace edyn {

struct settings {
    scalar fixed_dt {scalar(1.0 / 60)};
    bool paused {false};
};

}

#endif // EDYN_CONTEXT_SETTINGS_HPP
