#ifndef EDYN_NETWORKING_CLIENT_NETWORKING_CONTEXT_HPP
#define EDYN_NETWORKING_CLIENT_NETWORKING_CONTEXT_HPP

#include "edyn/networking/comp/networked_comp.hpp"
#include "edyn/util/entity_map.hpp"
#include "edyn/networking/packet/util/pool_snapshot.hpp"
#include <entt/entity/fwd.hpp>
#include <entt/signal/sigh.hpp>
#include <memory>

namespace edyn {

namespace packet {
    struct edyn_packet;
}

struct pool_snapshot;

void import_pool_client_default(entt::registry &, const pool_snapshot &);
void insert_entity_components_default(entt::registry &, entt::entity entity,
                                      std::vector<pool_snapshot> &pools);
void insert_transient_components_default(entt::registry &, entt::entity entity,
                                         std::vector<pool_snapshot> &pools);

struct client_networking_context {
    entt::entity client_entity {entt::null};
    entt::sparse_set owned_entities;

    edyn::entity_map entity_map;
    std::vector<entt::entity> created_entities;
    std::vector<entt::entity> destroyed_entities;
    bool importing_entities {false};

    double last_snapshot_time {0};
    double snapshot_rate {30};

    using request_entity_func_t = void(entt::entity);
    entt::sigh<request_entity_func_t> request_entity_signal;

    auto request_entity_sink() {
        return entt::sink{request_entity_signal};
    }

    using packet_observer_func_t = void(const packet::edyn_packet &);
    entt::sigh<packet_observer_func_t> packet_signal;

    auto packet_sink() {
        return entt::sink{packet_signal};
    }

    entt::sigh<void(void)> client_entity_assigned_signal;
    auto client_entity_assigned_sink() {
        return entt::sink{client_entity_assigned_signal};
    }

    using import_pool_func_t = decltype(&import_pool_client_default);
    import_pool_func_t import_pool_func {&import_pool_client_default};

    using insert_entity_components_func_t = decltype(&insert_entity_components_default);
    insert_entity_components_func_t insert_entity_components_func {&insert_entity_components_default};

    using insert_transient_components_func_t = decltype(&insert_transient_components_default);
    insert_transient_components_func_t insert_transient_components_func {&insert_transient_components_default};
};

}

#endif // EDYN_NETWORKING_CLIENT_NETWORKING_CONTEXT_HPP
