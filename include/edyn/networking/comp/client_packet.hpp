#ifndef EDYN_NETWORKING_COMP_CLIENT_PACKET_HPP
#define EDYN_NETWORKING_COMP_CLIENT_PACKET_HPP

#include "edyn/networking/packet/edyn_packet.hpp"

namespace edyn {

struct client_packet {
    edyn::packet::edyn_packet packet;
    double arrival_timestamp;
};

}

#endif // EDYN_NETWORKING_COMP_CLIENT_PACKET_HPP
