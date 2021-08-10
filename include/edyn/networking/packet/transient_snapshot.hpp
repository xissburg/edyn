#ifndef EDYN_NETWORKING_PACKET_TRANSIENT_SNAPSHOT_HPP
#define EDYN_NETWORKING_PACKET_TRANSIENT_SNAPSHOT_HPP

#include <vector>
#include <utility>
#include <edyn/comp/position.hpp>
#include <edyn/comp/orientation.hpp>
#include <edyn/comp/linvel.hpp>
#include <edyn/comp/angvel.hpp>
#include <entt/entity/fwd.hpp>

namespace edyn::packet {

struct transient_snapshot {
    std::vector<std::pair<entt::entity, position>> positions;
    std::vector<std::pair<entt::entity, orientation>> orientations;
    std::vector<std::pair<entt::entity, linvel>> linvels;
    std::vector<std::pair<entt::entity, angvel>> angvels;
};

template<typename Archive>
void serialize(Archive &archive, transient_snapshot &snapshot) {
    archive(snapshot.positions);
    archive(snapshot.orientations);
    archive(snapshot.linvels);
    archive(snapshot.angvels);
}

}

#endif // EDYN_NETWORKING_PACKET_TRANSIENT_SNAPSHOT_HPP
