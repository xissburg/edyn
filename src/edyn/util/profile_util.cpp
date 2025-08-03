#include "edyn/util/profile_util.hpp"
#include "edyn/context/profile.hpp"
#include "edyn/serialization/s11n.hpp"
#include "edyn/networking/packet/edyn_packet.hpp"
#include "edyn/serialization/memory_archive.hpp"
#include <entt/entity/registry.hpp>

namespace edyn {

#ifndef EDYN_DISABLE_PROFILING

void profile_on_packet_sent(entt::registry &registry, const packet::edyn_packet &packet) {
    auto data = std::vector<uint8_t>{};
    auto archive = edyn::memory_output_archive(data);
    archive(packet);

    auto &profile = registry.ctx().get<profile_network>();
    profile.sent += data.size();
}

void profile_on_packet_received(entt::registry &registry, const packet::edyn_packet &packet) {
    auto data = std::vector<uint8_t>{};
    auto archive = edyn::memory_output_archive(data);
    archive(packet);

    auto &profile = registry.ctx().get<profile_network>();
    profile.received += data.size();
}

void update_network_profiling(entt::registry &registry, double time) {
    auto &profile = registry.ctx().get<profile_network>();
    auto dt = time - profile.last_time;
    profile.incoming_rate = profile.received / dt;
    profile.outgoing_rate = profile.sent / dt;

    if (dt > profile.sample_length) {
        profile.last_time = time;
        profile.total_sent += profile.sent;
        profile.total_received += profile.received;
        profile.received = profile.sent = 0;
    }
}

#endif

}
