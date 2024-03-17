#ifndef EDYN_SIMULATION_ISLAND_MANAGER_HPP
#define EDYN_SIMULATION_ISLAND_MANAGER_HPP

#include <vector>
#include <entt/entity/fwd.hpp>
#include <entt/signal/sigh.hpp>
#include <entt/entity/sparse_set.hpp>

namespace edyn {

class island_manager {
    void init_new_nodes_and_edges();
    entt::entity create_island();
    void insert_to_island(entt::entity island_entity,
                          const std::vector<entt::entity> &nodes,
                          const std::vector<entt::entity> &edges);
    entt::entity merge_islands(const std::vector<entt::entity> &island_entities,
                               const std::vector<entt::entity> &new_nodes,
                               const std::vector<entt::entity> &new_edges);
    void split_islands();
    void wake_up_islands();

    bool could_go_to_sleep(entt::entity island_entity) const;
    void put_islands_to_sleep();

    void on_construct_graph_node(entt::registry &, entt::entity);
    void on_construct_graph_edge(entt::registry &, entt::entity);
    void on_destroy_graph_node(entt::registry &, entt::entity);
    void on_destroy_graph_edge(entt::registry &, entt::entity);
    void on_destroy_island_resident(entt::registry &, entt::entity);
    void on_destroy_multi_island_resident(entt::registry &, entt::entity);

public:
    island_manager(entt::registry &registry);
    ~island_manager();

    void put_to_sleep(entt::entity island_entity);
    void put_all_to_sleep();

    void update(double timestamp);

    void set_procedural(entt::entity entity, bool is_procedural);

    void set_last_time(double time) {
        m_last_time = time;
    }

private:
    entt::registry *m_registry;
    std::vector<entt::entity> m_new_graph_nodes;
    std::vector<entt::entity> m_new_graph_edges;
    entt::sparse_set m_islands_to_split;
    entt::sparse_set m_islands_to_wake_up;
    std::vector<entt::scoped_connection> m_connections;
    double m_last_time;
};

}

#endif // EDYN_SIMULATION_ISLAND_MANAGER_HPP
