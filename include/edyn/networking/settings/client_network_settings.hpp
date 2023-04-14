#ifndef EDYN_NETWORKING_SETTINGS_CLIENT_NETWORK_SETTINGS_HPP
#define EDYN_NETWORKING_SETTINGS_CLIENT_NETWORK_SETTINGS_HPP

#include "edyn/math/scalar.hpp"
#include <entt/entity/fwd.hpp>

namespace edyn {

using extrapolation_callback_t = void(*)(entt::registry &);

struct client_network_settings {
    double snapshot_rate {20};
    double round_trip_time {0};
    bool extrapolation_enabled {true};

    // Exponential decay parameter for discontinuity over time.
    scalar discontinuity_decay_rate {scalar(6)};

    // All actions older than this amount are deleted in every update.
    // The entire action history is included in every registry snapshot, thus
    // it is desirable to keep this low to minimize packet size. Though, a
    // longer action history decreases the chances of actions being lost. It
    // is sensible to increase it in case packet loss is high.
    double action_history_max_age {1.0};

    extrapolation_callback_t extrapolation_init_callback {nullptr};
    extrapolation_callback_t extrapolation_deinit_callback {nullptr};
    extrapolation_callback_t extrapolation_begin_callback {nullptr};
    extrapolation_callback_t extrapolation_finish_callback {nullptr};
    extrapolation_callback_t extrapolation_pre_step_callback {nullptr};
    extrapolation_callback_t extrapolation_post_step_callback {nullptr};
};

}

#endif // EDYN_NETWORKING_SETTINGS_CLIENT_NETWORK_SETTINGS_HPP
