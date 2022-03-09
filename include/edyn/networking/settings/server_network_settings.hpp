#ifndef EDYN_NETWORKING_SETTINGS_SERVER_NETWORK_SETTINGS_HPP
#define EDYN_NETWORKING_SETTINGS_SERVER_NETWORK_SETTINGS_HPP

namespace edyn {

struct server_network_settings {
    // Client playout delay buffer length will be calculated as the greatest
    // latency among all clients in its AABB of interest multiplied by this
    // value, which should be greater than 1, to ensure that all client
    // input will be applied with correct relative timing.
    double playout_delay_multiplier {1.2};

    // Prevent playout delay from getting too lengthy. If it gets maxed out due
    // to the large RTT of a client, packets received from that client will no
    // longer be delayed, they'll be applied immediately instead, which can lead
    // to jitter.
    double max_playout_delay {2};
};

}

#endif // EDYN_NETWORKING_SETTINGS_SERVER_NETWORK_SETTINGS_HPP
