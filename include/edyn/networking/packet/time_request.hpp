#ifndef EDYN_NETWORKING_PACKET_TIME_REQUEST_HPP
#define EDYN_NETWORKING_PACKET_TIME_REQUEST_HPP

namespace edyn::packet {

struct time_request {};

template<typename Archive>
void serialize(Archive &, time_request &) {}

}

#endif // EDYN_NETWORKING_PACKET_TIME_REQUEST_HPP
