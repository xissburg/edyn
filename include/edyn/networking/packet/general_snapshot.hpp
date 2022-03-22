#ifndef EDYN_NETWORKING_PACKET_GENERAL_SNAPSHOT_HPP
#define EDYN_NETWORKING_PACKET_GENERAL_SNAPSHOT_HPP

#include <vector>
#include "edyn/networking/util/registry_snapshot.hpp"

namespace edyn::packet {

/**
 * @brief A snapshot of a set of entities and components. It is used to share
 * updates to steady/non-transient components. It is the reliable counterpart
 * of `transient_snapshot`.
 */
struct general_snapshot : public registry_snapshot {
    double timestamp;
};

template<typename Archive>
void serialize(Archive &archive, general_snapshot &snapshot) {
    archive(snapshot.timestamp);
    archive(snapshot.entities);
    archive(snapshot.pools);
}

}

#endif // EDYN_NETWORKING_PACKET_GENERAL_SNAPSHOT_HPP
