#ifndef EDYN_CONTEXT_PROFILE_MACROS_HPP
#define EDYN_CONTEXT_PROFILE_MACROS_HPP

#include "edyn/time/time.hpp"

namespace edyn {

#ifndef EDYN_DISABLE_PROFILING

#define EDYN_PROFILE_BEGIN(time_var) \
    auto time_var = edyn::performance_time();

#define EDYN_PROFILE_MEASURE(time_var, profile, what) \
    profile.what = edyn::performance_time() - time_var; \
    time_var += profile.what;

#define EDYN_PROFILE_MEASURE_ACCUM(time_var, profile, what) \
    {\
        auto dt = edyn::performance_time() - time_var; \
        profile.what += dt; \
        time_var += dt; \
    }

#define EDYN_PROFILE_MEASURE_AVG(profile, what, count) \
    profile.what /= count;

#else

#undef EDYN_PROFILE_BEGIN
#define EDYN_PROFILE_BEGIN(...) ((void)0)

#undef EDYN_PROFILE_MEASURE
#define EDYN_PROFILE_MEASURE(...) ((void)0)

#undef EDYN_PROFILE_MEASURE_ACCUM
#define EDYN_PROFILE_MEASURE_ACCUM(...) ((void)0)

#undef EDYN_PROFILE_MEASURE_AVG
#define EDYN_PROFILE_MEASURE_AVG(...) ((void)0)

#endif

}

#endif // EDYN_CONTEXT_PROFILE_MACROS_HPP
