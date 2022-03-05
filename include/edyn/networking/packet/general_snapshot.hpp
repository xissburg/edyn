#ifndef EDYN_NETWORKING_PACKET_GENERAL_SNAPSHOT_HPP
#define EDYN_NETWORKING_PACKET_GENERAL_SNAPSHOT_HPP

#include <vector>
#include "edyn/networking/packet/util/pool_snapshot.hpp"

namespace edyn::packet {

struct general_snapshot {
    double timestamp;
    std::vector<pool_snapshot> pools;

    void convert_remloc(entity_map &emap) {
        for (auto &pool : pools) {
            pool.ptr->convert_remloc(emap);
        }
    }
};

template<typename Archive>
void serialize(Archive &archive, general_snapshot &snapshot) {
    archive(snapshot.timestamp);
    archive(snapshot.pools);
}

}

#endif // EDYN_NETWORKING_PACKET_GENERAL_SNAPSHOT_HPP
