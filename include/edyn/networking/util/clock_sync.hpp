#ifndef EDYN_NETWORKING_UTIL_CLOCK_SYNC_HPP
#define EDYN_NETWORKING_UTIL_CLOCK_SYNC_HPP

#include "edyn/config/config.h"
#include "edyn/networking/packet/edyn_packet.hpp"
#include "edyn/networking/packet/time_request.hpp"
#include "edyn/networking/packet/time_response.hpp"
#include "edyn/time/time.hpp"
#include <cstdlib>
#include <entt/signal/fwd.hpp>

namespace edyn {

enum class clock_sync_state {
    none, // No synchronization in course.
    time_req0, // First time request sent, waiting for response.
    time_req1, // Second time request sent, waiting for response.
    wait_next // Waiting to restart the process to acquire one more sample.
};

struct clock_sync_data {
    // Current state.
    clock_sync_state state {clock_sync_state::none};

    // Time when the last time request was sent.
    double time_req_timestamp;

    // Time request id which will be matched in a time response.
    uint32_t time_req_id;

    // Round-trip time of first time request.
    double rtt0;

    // Timestamp in the first time response.
    double remote_timestamp0;

    // Delay between time requests.
    double delay {1};

    // Time delta between local and remote clocks.
    double time_delta;

    // Maximum number of samples to be captured in one clock sync sequence.
    static constexpr auto max_time_delta_samples = 5;

    // Number of samples obtained so far.
    unsigned time_delta_sample_count {0};

    // Series of time delta samples to be averaged into a single value.
    double time_delta_samples[max_time_delta_samples];

    // Number of clock synchronizations performed.
    unsigned count {0};

    // Length of time between clock sychronizations, i.e. inverse of how often
    // clock syncs should be performed.
    double period {20.0 * 60.0};

    // Delegate invoked when the clock synchronization process needs to send a
    // packet, such as a time request.
    entt::delegate<void(const packet::edyn_packet &)> send_packet;
};

void clock_sync_process_time_response(clock_sync_data &clock_sync, const packet::time_response &res);
void update_clock_sync(clock_sync_data &clock_sync, double time, double avg_rtt);

}

#endif // EDYN_NETWORKING_UTIL_CLOCK_SYNC_HPP
