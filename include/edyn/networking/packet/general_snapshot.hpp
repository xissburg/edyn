#ifndef EDYN_NETWORKING_PACKET_GENERAL_SNAPSHOT_HPP
#define EDYN_NETWORKING_PACKET_GENERAL_SNAPSHOT_HPP

#include <vector>
#include "edyn/networking/util/pool_snapshot.hpp"

namespace edyn::packet {

/**
 * @brief A snapshot of a set of entities and components. It is the reliable
 * counterpart of `transient_snapshot`.
 */
struct general_snapshot {
    double timestamp;
    std::vector<entt::entity> entities;
    std::vector<pool_snapshot> pools;

    void convert_remloc(const entt::registry &registry, entity_map &emap) {
        for (auto &entity : entities) {
            entity = emap.at(entity);
        }

        for (auto &pool : pools) {
            pool.ptr->convert_remloc(registry, emap);
        }
    }
};

template<typename Archive>
void serialize(Archive &archive, general_snapshot &snapshot) {
    archive(snapshot.timestamp);
    archive(snapshot.entities);
    archive(snapshot.pools);
}

}

#endif // EDYN_NETWORKING_PACKET_GENERAL_SNAPSHOT_HPP
