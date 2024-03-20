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
#include "edyn/networking/extrapolation/extrapolation_modified_comp.hpp"
#include "edyn/networking/util/input_state_history.hpp"
#include "edyn/networking/util/pool_snapshot.hpp"
#include "edyn/parallel/message_dispatcher.hpp"
#include "edyn/core/entity_pair.hpp"
#include "edyn/replication/registry_operation.hpp"
#include "edyn/util/rigidbody.hpp"

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
    bool should_accumulate_discontinuities;
};

struct wake_up_residents {
    std::vector<entt::entity> residents;
};

struct change_rigidbody_kind {
    std::vector<std::pair<entt::entity, rigidbody_kind>> changes;
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
    bool query_procedural;
    bool query_non_procedural;
    bool query_islands;
};

struct query_aabb_of_interest_request {
    unsigned id;
    AABB aabb;
};

struct query_aabb_response {
    unsigned id;
    std::vector<entt::entity> procedural_entities;
    std::vector<entt::entity> non_procedural_entities;
    std::vector<entt::entity> island_entities;
};

struct set_extrapolator_context_settings {
    std::shared_ptr<input_state_history_reader> input_history;
    make_extrapolation_modified_comp_func_t *make_extrapolation_modified_comp;
};

struct paged_triangle_mesh_load_page {
    paged_triangle_mesh *trimesh;
    size_t mesh_index;
};

}

#endif // EDYN_PARALLEL_MESSAGE_HPP
