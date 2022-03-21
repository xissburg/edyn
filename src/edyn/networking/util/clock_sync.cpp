#include "edyn/networking/util/clock_sync.hpp"

namespace edyn {

static void start_clock_sync(clock_sync_data &clock_sync, double time) {
    clock_sync.state = clock_sync_state::time_req0;
    clock_sync.time_req_timestamp = time;
    clock_sync.time_req_id = static_cast<uint32_t>(std::rand());

    auto packet = packet::time_request{clock_sync.time_req_id};
    clock_sync.send_packet(packet::edyn_packet{packet});
}

void clock_sync_process_time_response(clock_sync_data &clock_sync, const packet::time_response &res) {
    if (clock_sync.state != clock_sync_state::time_req0 &&
        clock_sync.state != clock_sync_state::time_req1) {
        return;
    }

    if (res.id != clock_sync.time_req_id) {
        return;
    }

    auto time = performance_time();

    if (clock_sync.state == clock_sync_state::time_req0) {
        clock_sync.state = clock_sync_state::time_req1;
        clock_sync.rtt0 = time - clock_sync.time_req_timestamp;
        clock_sync.remote_timestamp0 = res.timestamp;

        clock_sync.time_req_timestamp = time;
        clock_sync.time_req_id = static_cast<uint32_t>(std::rand());

        auto packet = packet::time_request{clock_sync.time_req_id};
        clock_sync.send_packet(packet::edyn_packet{packet});
    } else {
        auto rtt0 = clock_sync.rtt0;
        auto rtt1 = time - clock_sync.time_req_timestamp;
        auto rtt = (rtt0 + rtt1) / 2;
        auto remote_dt = res.timestamp - clock_sync.remote_timestamp0;

        // If RTT of time requests differ by more than 10%, start again from zero.
        // If delta between remote timestamps differ from average time request
        // RTT by more than 10%, restart from zero.
        if (std::abs(rtt0 - rtt1) > rtt0 * 0.1 || std::abs(rtt - remote_dt) > rtt * 0.1) {
            start_clock_sync(clock_sync, time);
            return;
        }

        auto current_remote_time = res.timestamp + rtt1 / 2;
        auto time_delta = time - current_remote_time;
        clock_sync.time_delta_samples[clock_sync.time_delta_sample_count++] = time_delta;
        clock_sync.time_req_timestamp = time;

        if (clock_sync.time_delta_sample_count == clock_sync.max_time_delta_samples) {
            clock_sync.state = clock_sync_state::none;
            clock_sync.time_delta = 0;

            for (auto dt : clock_sync.time_delta_samples) {
                clock_sync.time_delta += dt;
            }

            clock_sync.time_delta /= clock_sync.max_time_delta_samples;
            clock_sync.time_delta_sample_count = 0;
            clock_sync.count++;
        } else {
            clock_sync.state = clock_sync_state::wait_next;
        }
    }
}

void update_clock_sync(clock_sync_data &clock_sync, double time, double avg_rtt) {
    switch (clock_sync.state) {
    case clock_sync_state::none: {
        if (clock_sync.count == 0 || time - clock_sync.time_req_timestamp > clock_sync.period) {
            start_clock_sync(clock_sync, time);
        }
    } break;
    case clock_sync_state::time_req0:
    case clock_sync_state::time_req1: {
        if (time - clock_sync.time_req_timestamp > avg_rtt * 1.4) {
            // Assume packet lost, start again from zero.
            start_clock_sync(clock_sync, time);
        }
    } break;
    case clock_sync_state::wait_next: {
        if (time - clock_sync.time_req_timestamp > clock_sync.delay) {
            start_clock_sync(clock_sync, time);
        }
    } break;
    }
}

}
