#ifndef EDYN_PARALLEL_ISLAND_COORDINATOR_HPP
#define EDYN_PARALLEL_ISLAND_COORDINATOR_HPP

#include <vector>
#include <memory>
#include <unordered_map>
#include <entt/entity/fwd.hpp>
#include <entt/signal/sigh.hpp>
#include "edyn/comp/island.hpp"
#include "edyn/config/config.h"
#include "edyn/parallel/component_index_source.hpp"
#include "edyn/parallel/island_worker_context.hpp"
#include "edyn/parallel/message.hpp"
#include "edyn/util/registry_operation.hpp"
#include "edyn/util/registry_operation_builder.hpp"

namespace edyn {

class island_worker;

/**
 * Manages all simulation islands. Creates and destroys island workers as necessary
 * and synchronizes the workers and the main registry.
 */
class island_coordinator final {

    void init_new_nodes_and_edges();
    void init_new_non_procedural_node(entt::entity);
    entt::entity create_island(double timestamp, bool sleeping,
                               const std::vector<entt::entity> &nodes,
                               const std::vector<entt::entity> &edges);
    void insert_to_island(island_worker_context &ctx,
                          const std::vector<entt::entity> &nodes,
                          const std::vector<entt::entity> &edges);
    void insert_to_island(entt::entity island_entity,
                          const std::vector<entt::entity> &nodes,
                          const std::vector<entt::entity> &edges);
    entt::entity merge_islands(const std::vector<entt::entity> &island_entities,
                               const std::vector<entt::entity> &new_nodes,
                               const std::vector<entt::entity> &new_edges);
    void split_islands();
    void split_island(entt::entity);
    void refresh_dirty_entities();
    bool should_split_island(entt::entity source_island_entity);
    void sync();

public:
    island_coordinator(island_coordinator const&) = delete;
    island_coordinator operator=(island_coordinator const&) = delete;
    island_coordinator(entt::registry &);
    ~island_coordinator();

    void on_construct_graph_node(entt::registry &, entt::entity);
    void on_construct_graph_edge(entt::registry &, entt::entity);

    void on_destroy_graph_node(entt::registry &, entt::entity);
    void on_destroy_graph_edge(entt::registry &, entt::entity);

    void on_destroy_island_resident(entt::registry &, entt::entity);
    void on_destroy_multi_island_resident(entt::registry &, entt::entity);
    void on_island_reg_ops(entt::entity, const msg::island_reg_ops &);
    void on_split_island(entt::entity, const msg::split_island &);

    void on_destroy_contact_manifold(entt::registry &, entt::entity);

    void update();

    void set_paused(bool);
    void step_simulation();

    template<typename... Component>
    void refresh(entt::entity entity);

    void set_center_of_mass(entt::entity entity, const vector3 &com);

    // Call when settings have changed in the registry's context. It will
    // propagate changes to island workers.
    void settings_changed();

    void material_table_changed();

    void create_island(std::vector<entt::entity> nodes, bool sleeping = false);

    auto contact_started_sink() {
        return entt::sink{m_contact_started_signal};
    }

    auto contact_ended_sink() {
        return entt::sink{m_contact_ended_signal};
    }

    auto contact_point_created_sink() {
        return entt::sink{m_contact_point_created_signal};
    }

    auto contact_point_destroyed_sink() {
        return entt::sink{m_contact_point_destroyed_signal};
    }

    template<typename Message, typename... Args>
    void send_island_message(entt::entity island_entity, Args &&... args) {
        auto &ctx = m_island_ctx_map.at(island_entity);
        ctx->send<Message>(std::forward<Args>(args)...);
    }

    void wake_up_island(entt::entity island_entity) {
        auto &ctx = m_island_ctx_map.at(island_entity);
        ctx->send<msg::wake_up_island>();
    }

private:
    entt::registry *m_registry;
    std::unordered_map<entt::entity, std::unique_ptr<island_worker_context>> m_island_ctx_map;

    entt::sigh<void(entt::entity)> m_contact_started_signal;
    entt::sigh<void(entt::entity)> m_contact_ended_signal;
    entt::sigh<void(entt::entity, contact_manifold::contact_id_type)> m_contact_point_created_signal;
    entt::sigh<void(entt::entity, contact_manifold::contact_id_type)> m_contact_point_destroyed_signal;

    std::vector<entt::entity> m_new_graph_nodes;
    std::vector<entt::entity> m_new_graph_edges;
    std::vector<entt::entity> m_islands_to_split;

    bool m_importing {false};
    double m_timestamp;
};

template<typename... Component>
void island_coordinator::refresh(entt::entity entity) {
    static_assert(sizeof...(Component) > 0);

#ifdef EDYN_DEBUG
    auto &index_source = m_registry->ctx<settings>().index_source;
    auto contains_unknown = ((index_source->index_of<Component>() == SIZE_MAX) || ...);
    EDYN_ASSERT(!contains_unknown);
#endif

    if (m_registry->any_of<island_resident>(entity)) {
        auto &resident = m_registry->get<island_resident>(entity);

        if (resident.island_entity != entt::null) {
            auto &ctx = m_island_ctx_map.at(resident.island_entity);
            (ctx->m_op_builder->replace<Component>(*m_registry, entity), ...);
        }
    } else if (m_registry->any_of<multi_island_resident>(entity)) {
        auto &resident = m_registry->get<multi_island_resident>(entity);

        for (auto island_entity : resident.island_entities) {
            auto &ctx = m_island_ctx_map.at(island_entity);
            (ctx->m_op_builder->replace<Component>(*m_registry, entity), ...);
        }
    }
}

}

#endif // EDYN_PARALLEL_ISLAND_COORDINATOR_HPP
