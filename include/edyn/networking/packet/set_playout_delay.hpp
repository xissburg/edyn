#ifndef EDYN_NETWORKING_PACKET_SET_PLAYOUT_DELAY_HPP
#define EDYN_NETWORKING_PACKET_SET_PLAYOUT_DELAY_HPP

namespace edyn::packet {

struct set_playout_delay {
    double value;
};

template<typename Archive>
void serialize(Archive &archive, set_playout_delay &delay) {
    archive(delay.value);
}

}

#endif // EDYN_NETWORKING_PACKET_SET_PLAYOUT_DELAY_HPP
