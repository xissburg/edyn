#ifndef EDYN_NETWORKING_PACKET_TRANSIENT_SNAPSHOT_HPP
#define EDYN_NETWORKING_PACKET_TRANSIENT_SNAPSHOT_HPP

#include <memory>
#include <edyn/networking/packet/util/pool_snapshot.hpp>

namespace edyn::packet {

struct transient_snapshot {
    std::vector<std::unique_ptr<pool_snapshot_base>> pools;
};

template<typename Archive>
void serialize(Archive &archive, transient_snapshot &snapshot) {
    archive(snapshot.pools);
}

}

#endif // EDYN_NETWORKING_PACKET_TRANSIENT_SNAPSHOT_HPP
