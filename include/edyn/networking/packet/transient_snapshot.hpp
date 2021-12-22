#ifndef EDYN_NETWORKING_PACKET_TRANSIENT_SNAPSHOT_HPP
#define EDYN_NETWORKING_PACKET_TRANSIENT_SNAPSHOT_HPP

#include <vector>
#include "edyn/networking/packet/util/pool_snapshot.hpp"

namespace edyn::packet {

struct transient_snapshot {
    std::vector<pool_snapshot> pools;
    std::vector<uint8_t> user_data;
};

template<typename Archive>
void serialize(Archive &archive, transient_snapshot &snapshot) {
    archive(snapshot.pools);
    archive(snapshot.user_data);
}

}

#endif // EDYN_NETWORKING_PACKET_TRANSIENT_SNAPSHOT_HPP
