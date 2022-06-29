#ifndef EDYN_MATH_SCALAR_HPP
#define EDYN_MATH_SCALAR_HPP

#include <float.h>
#include "edyn/build_settings.h"

namespace edyn {

#ifdef EDYN_DOUBLE_PRECISION
using scalar = double;
#define EDYN_EPSILON (DBL_EPSILON)
#define EDYN_SCALAR_MIN (DBL_MIN)
#define EDYN_SCALAR_MAX (DBL_MAX)
#else
using scalar = float;
#define EDYN_EPSILON (FLT_EPSILON)
#define EDYN_SCALAR_MIN (FLT_MIN)
#define EDYN_SCALAR_MAX (FLT_MAX)
#endif

}

#endif // EDYN_MATH_SCALAR_HPP
