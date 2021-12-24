#ifndef EDYN_NETWORKING_PACKET_GENERAL_SNAPSHOT_HPP
#define EDYN_NETWORKING_PACKET_GENERAL_SNAPSHOT_HPP

#include <vector>
#include "edyn/networking/packet/util/pool_snapshot.hpp"

namespace edyn::packet {

struct general_snapshot {
    std::vector<pool_snapshot> pools;
};

template<typename Archive>
void serialize(Archive &archive, general_snapshot &snapshot) {
    archive(snapshot.pools);
}

}

#endif // EDYN_NETWORKING_PACKET_GENERAL_SNAPSHOT_HPP
