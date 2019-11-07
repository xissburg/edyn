#ifndef EDYN_MATH_SCALAR_HPP
#define EDYN_MATH_SCALAR_HPP

#include <float.h>
#include "build_settings.h"

namespace edyn {

#if EDYN_DOUBLE_PRECISION
using scalar = double;
#define EDYN_EPSILON (DBL_EPSILON)
#else
using scalar = float;
#define EDYN_EPSILON (FLT_EPSILON)
#endif

}

#endif // EDYN_MATH_SCALAR_HPP