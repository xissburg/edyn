#ifndef EDYN_MATH_SCALAR_HPP
#define EDYN_MATH_SCALAR_HPP

#include "build_settings.h"

namespace edyn {

#if EDYN_DOUBLE_PRECISION
using scalar = double;
#else
using scalar = float;
#endif

}

#endif // EDYN_MATH_SCALAR_HPP