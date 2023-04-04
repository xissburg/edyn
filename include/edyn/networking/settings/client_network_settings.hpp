#ifndef EDYN_NETWORKING_SETTINGS_CLIENT_NETWORK_SETTINGS_HPP
#define EDYN_NETWORKING_SETTINGS_CLIENT_NETWORK_SETTINGS_HPP

#include "edyn/math/scalar.hpp"

namespace edyn {

struct client_network_settings {
    double snapshot_rate {20};
    double round_trip_time {0};
    bool extrapolation_enabled {true};

    // The discontinuity error will be multiplied by this value after every
    // step. That means this value is sensitive to the fixed delta time since
    // a lower delta time means higher step rate, thus faster decay.
    scalar discontinuity_decay_rate {scalar(0.9)};

    // All actions older than this amount are deleted in every update.
    // The entire action history is included in every registry snapshot, thus
    // it is desirable to keep this low to minimize packet size. Though, a
    // longer action history decreases the chances of actions being lost. It
    // is sensible to increase it in case packet loss is high.
    double action_history_max_age {1.0};
};

}

#endif // EDYN_NETWORKING_SETTINGS_CLIENT_NETWORK_SETTINGS_HPP
