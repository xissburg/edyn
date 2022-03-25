#ifndef EDYN_NETWORKING_PACKET_SET_PLAYOUT_DELAY_HPP
#define EDYN_NETWORKING_PACKET_SET_PLAYOUT_DELAY_HPP

namespace edyn::packet {

/**
 * @brief Sent to clients when their playout delay changes in the server.
 */
struct set_playout_delay {
    double value;
};

template<typename Archive>
void serialize(Archive &archive, set_playout_delay &delay) {
    archive(delay.value);
}

}

#endif // EDYN_NETWORKING_PACKET_SET_PLAYOUT_DELAY_HPP
