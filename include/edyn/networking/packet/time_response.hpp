#ifndef EDYN_NETWORKING_PACKET_TIME_RESPONSE_HPP
#define EDYN_NETWORKING_PACKET_TIME_RESPONSE_HPP

#include <cstdint>

namespace edyn::packet {

struct time_response {
    uint32_t id;
    double timestamp;
};

template<typename Archive>
void serialize(Archive &archive, time_response &res) {
    archive(res.id);
    archive(res.timestamp);
}

}

#endif // EDYN_NETWORKING_PACKET_TIME_RESPONSE_HPP
