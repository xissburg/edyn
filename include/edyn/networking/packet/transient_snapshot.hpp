#ifndef EDYN_NETWORKING_PACKET_TRANSIENT_SNAPSHOT_HPP
#define EDYN_NETWORKING_PACKET_TRANSIENT_SNAPSHOT_HPP

#include <vector>
#include "edyn/networking/packet/util/pool_snapshot.hpp"

namespace edyn::packet {

struct transient_snapshot {
    double timestamp;
    std::vector<pool_snapshot> pools;

    auto get_entities() const {
        entt::sparse_set entities;

        for (auto &pool : pools) {
            for (auto entity : pool.ptr->get_entities()) {
                if (!entities.contains(entity)) {
                    entities.emplace(entity);
                }
            }
        }

        return entities;
    }

    void convert_remloc(entity_map &emap) {
        for (auto &pool : pools) {
            pool.ptr->convert_remloc(emap);
        }
    }
};

template<typename Archive>
void serialize(Archive &archive, transient_snapshot &snapshot) {
    archive(snapshot.timestamp);
    archive(snapshot.pools);
}

}

#endif // EDYN_NETWORKING_PACKET_TRANSIENT_SNAPSHOT_HPP
