#include "edyn/networking/util/server_validate_packet.hpp"
#include "edyn/networking/util/pool_snapshot.hpp"
#include "edyn/networking/comp/remote_client.hpp"
#include <entt/entity/registry.hpp>
#include <vector>
#include <cmath>

namespace edyn {

static bool validate_pool_snapshots(const entt::registry &registry, entt::entity client_entity,
                                    const std::vector<entt::entity> &pool_entities,
                                    std::vector<pool_snapshot> &pools) {
    auto &client = registry.get<remote_client>(client_entity);

    for (auto remote_entity : pool_entities) {
        if (!client.entity_map.contains(remote_entity)) {
            return false;
        }
    }

    for (auto &pool : pools) {
        if (!pool.ptr->validate(registry, pool_entities, client.entity_map)) {
            return false;
        }
    }

    return true;
}

bool server_validate_packet(const entt::registry &registry, entt::entity client_entity, packet::entity_response &packet) {
    return validate_pool_snapshots(registry, client_entity, packet.entities, packet.pools);
}

bool server_validate_packet(const entt::registry &registry, entt::entity client_entity, packet::create_entity &packet) {
    if (!std::isfinite(packet.timestamp)) {
        return false;
    }

    return validate_pool_snapshots(registry, client_entity, packet.entities, packet.pools);
}

bool server_validate_packet(const entt::registry &registry, entt::entity client_entity, packet::destroy_entity &packet) {
    if (!std::isfinite(packet.timestamp)) {
        return false;
    }

    for (auto entity : packet.entities) {
        if (!registry.valid(entity)) {
            return false;
        }
    }

    return true;
}

bool server_validate_packet(const entt::registry &registry, entt::entity client_entity, packet::update_entity_map &packet) {
    if (!std::isfinite(packet.timestamp)) {
        return false;
    }

    for (auto &pair : packet.pairs) {
        auto local_entity = pair.first;
        if (!registry.valid(local_entity)) {
            return false;
        }
    }

    return true;
}

bool server_validate_packet(const entt::registry &registry, entt::entity client_entity, packet::client_created &packet) {
    return false;
}

bool server_validate_packet(const entt::registry &registry, entt::entity client_entity, packet::set_playout_delay &packet) {
    return false;
}

bool server_validate_packet(const entt::registry &registry, entt::entity client_entity, packet::time_request &packet) {
    return true;
}

bool server_validate_packet(const entt::registry &registry, entt::entity client_entity, packet::time_response &packet) {
    return std::isfinite(packet.timestamp);
}

bool server_validate_packet(const entt::registry &registry, entt::entity client_entity, packet::entity_request &packet) {
    for (auto entity : packet.entities) {
        if (!registry.valid(entity)) {
            return false;
        }
    }

    return true;
}

bool server_validate_packet(const entt::registry &registry, entt::entity client_entity, packet::transient_snapshot &packet) {
    if (!std::isfinite(packet.timestamp)) {
        return false;
    }

    return validate_pool_snapshots(registry, client_entity, packet.entities, packet.pools);
}

bool server_validate_packet(const entt::registry &registry, entt::entity client_entity, packet::general_snapshot &packet) {
    if (!std::isfinite(packet.timestamp)) {
        return false;
    }

    return validate_pool_snapshots(registry, client_entity, packet.entities, packet.pools);
}

}
