#ifndef EDYN_UTIL_PROFILE_UTIL_HPP
#define EDYN_UTIL_PROFILE_UTIL_HPP

#include "edyn/time/time.hpp"
#include <entt/entity/fwd.hpp>

namespace edyn {

#ifndef EDYN_DISABLE_PROFILING

#define EDYN_PROFILE_BEGIN(time_var) \
    auto time_var = edyn::performance_time();

#define EDYN_PROFILE_MEASURE(time_var, profile, what) \
    profile.what = edyn::performance_time() - time_var; \
    time_var += profile.what;

#define EDYN_PROFILE_MEASURE_ACCUM(time_var, profile, what) \
    { \
        auto t1 = edyn::performance_time(); \
        profile.what += t1 - time_var; \
        time_var = t1; \
    }

#define EDYN_PROFILE_MEASURE_AVG(profile, what, count) \
    profile.what /= count;

namespace packet {
    struct edyn_packet;
}

void profile_on_packet_sent(entt::registry &registry, const packet::edyn_packet &packet);
void profile_on_packet_received(entt::registry &registry, const packet::edyn_packet &packet);
void update_network_profiling(entt::registry &registry, double time);

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

#endif // EDYN_UTIL_PROFILE_UTIL_HPP
