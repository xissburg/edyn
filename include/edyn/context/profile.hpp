#ifndef EDYN_CONTEXT_PROFILE_HPP
#define EDYN_CONTEXT_PROFILE_HPP

#include "edyn/math/scalar.hpp"

namespace edyn {

struct profile_timers {
    scalar raycasts {};
    scalar broadphase {};
    scalar islands {};
    scalar narrowphase {};
    scalar step {}; // just the physics.
    scalar full_step {}; // including step callbacks, the whole operation.
    scalar restitution {};
    scalar prepare_constraints {};
    scalar solve_islands {};
    scalar apply_results {};
};

struct profile_counters {
    unsigned constraint_rows;
};

struct profile_network {
    unsigned sent;
    unsigned received;
    unsigned total_sent;
    unsigned total_received;
    scalar incoming_rate;
    scalar outgoing_rate;
    double sample_length {2};
    double last_time;
};

}

#endif // EDYN_CONTEXT_PROFILE_HPP
