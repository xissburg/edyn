#ifndef EDYN_NETWORKING_PACKET_TRANSIENT_SNAPSHOT_HPP
#define EDYN_NETWORKING_PACKET_TRANSIENT_SNAPSHOT_HPP

#include <vector>
#include "edyn/collision/contact_manifold.hpp"
#include "edyn/networking/packet/util/pool_snapshot.hpp"

namespace edyn::packet {

struct transient_snapshot {
    std::vector<pool_snapshot> pools;
    std::vector<uint8_t> user_data;
    std::vector<contact_manifold> manifolds;

    void convert_remloc(entity_map &emap) {
        for (auto &pool : pools) {
            pool.ptr->convert_remloc(emap);
        }

        auto remove_it = std::remove_if(manifolds.begin(), manifolds.end(), [&] (contact_manifold &manifold) {
            if (!emap.has_rem(manifold.body[0]) || !emap.has_rem(manifold.body[1])) {
                return true;
            }

            manifold.body[0] = emap.remloc(manifold.body[0]);
            manifold.body[1] = emap.remloc(manifold.body[1]);
            return false;
        });
        manifolds.erase(remove_it, manifolds.end());
    }
};

template<typename Archive>
void serialize(Archive &archive, transient_snapshot &snapshot) {
    archive(snapshot.pools);
    archive(snapshot.manifolds);
    archive(snapshot.user_data);
}

}

#endif // EDYN_NETWORKING_PACKET_TRANSIENT_SNAPSHOT_HPP
