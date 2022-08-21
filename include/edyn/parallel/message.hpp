#ifndef EDYN_PARALLEL_MESSAGE_HPP
#define EDYN_PARALLEL_MESSAGE_HPP

#include <entt/entity/fwd.hpp>
#include "edyn/collision/raycast.hpp"
#include "edyn/comp/aabb.hpp"
#include "edyn/comp/island.hpp"
#include "edyn/context/registry_operation_context.hpp"
#include "edyn/math/vector3.hpp"
#include "edyn/context/settings.hpp"
#include "edyn/dynamics/material_mixing.hpp"
#include "edyn/networking/util/pool_snapshot.hpp"
#include "edyn/parallel/message_dispatcher.hpp"
#include "edyn/core/entity_pair.hpp"
#include "edyn/replication/registry_operation.hpp"

namespace edyn::msg {

struct set_paused {
    bool paused;
};

struct set_settings {
    edyn::settings settings;
};

struct set_registry_operation_context {
    edyn::registry_operation_context ctx;
};

struct set_material_table {
    edyn::material_mix_table table;
};

struct step_simulation {};

struct set_com {
    entt::entity entity;
    vector3 com;
};

/**
 * Message sent by worker to the main thread after every step of the simulation
 * containing everything that changed since the previous update.
 */
struct step_update {
    registry_operation ops;
    double timestamp;
};

/**
 * Message sent by the stepper to a worker asking entities and components
 * to be created, destroyed or updated.
 */
struct update_entities {
    registry_operation ops;
};

struct apply_network_pools {
    std::vector<entt::entity> entities;
    std::vector<pool_snapshot> pools;
};

struct raycast_request {
    unsigned int id;
    vector3 p0, p1;
    std::vector<entt::entity> ignore_entities;
};

struct raycast_response {
    unsigned int id;
    raycast_result result;
};

struct query_aabb_request {
    unsigned id;
    AABB aabb;
    bool islands_only;
};

struct query_aabb_response {
    unsigned id;
    std::vector<entt::entity> entities;
};

}

#endif // EDYN_PARALLEL_MESSAGE_HPP
