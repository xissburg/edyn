#ifndef EDYN_NETWORKING_PACKET_TIME_REQUEST_HPP
#define EDYN_NETWORKING_PACKET_TIME_REQUEST_HPP

#include <cstdint>

namespace edyn::packet {

/**
 * @brief Time request for clock synchronization purposes.
 */
struct time_request {
    uint32_t id;
};

template<typename Archive>
void serialize(Archive &archive, time_request &req) {
    archive(req.id);
}

}

#endif // EDYN_NETWORKING_PACKET_TIME_REQUEST_HPP
