#ifndef EDYN_PARALLEL_ISLAND_COORDINATOR_HPP
#define EDYN_PARALLEL_ISLAND_COORDINATOR_HPP

#include <vector>
#include <memory>
#include <unordered_map>
#include <entt/fwd.hpp>
#include "edyn/math/scalar.hpp"
#include "edyn/parallel/island_worker_context.hpp"

namespace edyn {

class island_worker;
class registry_delta;

/**
 * Manages all simulation islands. Creates and destroys island workers as necessary
 * and synchonizes the workers and the main registry.
 */
class island_coordinator final {

    void init_new_island_nodes();
    void init_new_non_procedural_island_node(entt::entity);
    entt::entity create_island(double timestamp);
    entt::entity merge_islands(const entity_set &island_entities,
                               const entity_set &new_entities);
    void split_islands();
    void split_island(entt::entity);
    void wake_up_island(entt::entity);
    void refresh_dirty_entities();
    bool should_split_island(const island_topology &);
    void sync();

    void validate();

public:
    island_coordinator(entt::registry &);
    ~island_coordinator();

    void on_destroy_island_node_parent(entt::registry &, entt::entity);

    void on_construct_island_node(entt::registry &, entt::entity);
    void on_destroy_island_node(entt::registry &, entt::entity);
    void on_construct_island_container(entt::registry &, entt::entity);
    void on_destroy_island_container(entt::registry &, entt::entity);
    void on_registry_delta(entt::entity, const registry_delta &);
    
    void on_construct_constraint(entt::registry &, entt::entity);

    void update();

    void set_paused(bool);
    void step_simulation();

    template<typename... Component>
    void refresh(entt::entity entity) {
        static_assert(sizeof...(Component) > 0);
        auto &container = m_registry->get<island_container>(entity);
        for (auto island_entity : container.entities) {
            auto &ctx = m_island_ctx_map.at(island_entity);
            ctx->m_delta_builder.updated<Component...>(entity, *m_registry);
        }
    }

    scalar m_fixed_dt {1.0/60};

private:
    entt::registry *m_registry;
    std::unordered_map<entt::entity, std::unique_ptr<island_worker_context>> m_island_ctx_map;

    std::vector<entt::entity> m_new_island_nodes;
    entity_set m_islands_to_split;
    bool m_importing_delta {false};
    bool m_paused {false};
};

}

#endif // EDYN_PARALLEL_ISLAND_COORDINATOR_HPP