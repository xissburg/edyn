#ifndef EDYN_NETWORKING_SETTINGS_SERVER_NETWORK_SETTINGS_HPP
#define EDYN_NETWORKING_SETTINGS_SERVER_NETWORK_SETTINGS_HPP

namespace edyn {

struct server_network_settings {
    // Client playout delay buffer length will be calculated as the greatest
    // latency among all clients in its AABB of interest multiplied by this
    // value, which should be greater than 1, to ensure that all client
    // input will be applied with correct relative timing.
    double playout_delay_multiplier {1.2};
};

}

#endif // EDYN_NETWORKING_SETTINGS_SERVER_NETWORK_SETTINGS_HPP
