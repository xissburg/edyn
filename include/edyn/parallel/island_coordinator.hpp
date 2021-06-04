#ifndef EDYN_PARALLEL_ISLAND_COORDINATOR_HPP
#define EDYN_PARALLEL_ISLAND_COORDINATOR_HPP

#include <vector>
#include <memory>
#include <unordered_map>
#include <entt/fwd.hpp>
#include "edyn/math/scalar.hpp"
#include "edyn/comp/island.hpp"
#include "edyn/parallel/island_delta.hpp"
#include "edyn/parallel/island_worker_context.hpp"
#include "edyn/parallel/island_delta_builder.hpp"
#include "edyn/parallel/message.hpp"

namespace edyn {

class island_worker;
class island_delta;

/**
 * Manages all simulation islands. Creates and destroys island workers as necessary
 * and synchronizes the workers and the main registry.
 */
class island_coordinator final {

    void init_new_nodes_and_edges();
    void init_new_non_procedural_node(entt::entity);
    entt::entity create_island(double timestamp, bool sleeping);
    void insert_to_island(entt::entity island_entity,
                          const std::vector<entt::entity> &nodes,
                          const std::vector<entt::entity> &edges);
    entt::entity merge_islands(const std::vector<entt::entity> &island_entities,
                               const std::vector<entt::entity> &new_nodes,
                               const std::vector<entt::entity> &new_edges);
    void split_islands();
    void split_island(entt::entity);
    void wake_up_island(entt::entity);
    void refresh_dirty_entities();
    bool should_split_island(entt::entity source_island_entity);
    void sync();

public:
    island_coordinator(entt::registry &);
    ~island_coordinator();

    void on_construct_graph_node(entt::registry &, entt::entity);
    void on_construct_graph_edge(entt::registry &, entt::entity);

    void on_destroy_graph_node(entt::registry &, entt::entity);
    void on_destroy_graph_edge(entt::registry &, entt::entity);

    void on_destroy_island_resident(entt::registry &, entt::entity);
    void on_destroy_multi_island_resident(entt::registry &, entt::entity);
    void on_island_delta(entt::entity, const island_delta &);
    void on_split_island(entt::entity, const msg::split_island &);

    void on_destroy_contact_manifold(entt::registry &, entt::entity);

    void update();

    void set_paused(bool);
    void step_simulation();

    template<typename... Component>
    void refresh(entt::entity entity);

    void set_fixed_dt(scalar dt);

    // Call when settings have changed in the registry's context. It will
    // propagate changes to island workers.
    void settings_changed();

private:
    entt::registry *m_registry;
    std::unordered_map<entt::entity, std::unique_ptr<island_worker_context>> m_island_ctx_map;

    std::vector<entt::entity> m_new_graph_nodes;
    std::vector<entt::entity> m_new_graph_edges;
    std::vector<entt::entity> m_islands_to_split;

    bool m_importing_delta {false};
    double m_timestamp;
};

template<typename... Component>
void island_coordinator::refresh(entt::entity entity) {
    static_assert(sizeof...(Component) > 0);

    if (m_registry->has<island_resident>(entity)) {
        auto &resident = m_registry->get<island_resident>(entity);
        auto &ctx = m_island_ctx_map.at(resident.island_entity);
        ctx->m_delta_builder->updated<Component...>(entity, *m_registry);
    } else {
        auto &resident = m_registry->get<multi_island_resident>(entity);
        for (auto island_entity : resident.island_entities) {
            auto &ctx = m_island_ctx_map.at(island_entity);
            ctx->m_delta_builder->updated<Component...>(entity, *m_registry);
        }
    }
}

}

#endif // EDYN_PARALLEL_ISLAND_COORDINATOR_HPP
