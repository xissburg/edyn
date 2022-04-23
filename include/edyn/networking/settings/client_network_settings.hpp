#ifndef EDYN_NETWORKING_SETTINGS_CLIENT_NETWORK_SETTINGS_HPP
#define EDYN_NETWORKING_SETTINGS_CLIENT_NETWORK_SETTINGS_HPP

#include "edyn/math/scalar.hpp"

namespace edyn {

struct client_network_settings {
    double snapshot_rate {20};
    double round_trip_time {0};
    bool extrapolation_enabled {true};
    unsigned max_concurrent_extrapolations {2};

    // The discontinuity error will be multiplied by this value after every
    // step. That means this value is sensitive to the fixed delta time since
    // a lower delta time means higher step rate, thus faster decay.
    scalar discontinuity_decay_rate {scalar(0.9)};

    /**
     * Even though the timestamp of a registry snapshot lies right after the time
     * an action happenend, it is possible that the action wasn't still applied
     * in the server side at the time the snapshot was generated. Perhaps the
     * action was applied at the same time the snapshot was generated and then
     * its effects were only visible in the next update, which will cause a glitch
     * on client-side extrapolation because the action will not be applied initially
     * and the initial state does not include the effects of the action because
     * it wasn't applied in the server at the time the snapshot was generated.
     * All actions that happened before the snapshot time within this threshold
     * will be applied at the start of an extrapolation.
     */
    double action_time_threshold {0.06};
};

}

#endif // EDYN_NETWORKING_SETTINGS_CLIENT_NETWORK_SETTINGS_HPP
