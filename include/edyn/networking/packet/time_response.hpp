#ifndef EDYN_NETWORKING_PACKET_TIME_RESPONSE_HPP
#define EDYN_NETWORKING_PACKET_TIME_RESPONSE_HPP

namespace edyn::packet {

struct time_response {
    double timestamp;
};

template<typename Archive>
void serialize(Archive &archive, time_response &res) {
    archive(res.timestamp);
}

}

#endif // EDYN_NETWORKING_PACKET_TIME_RESPONSE_HPP
