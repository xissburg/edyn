#ifndef EDYN_PARALLEL_ISLAND_MANAGER_HPP
#define EDYN_PARALLEL_ISLAND_MANAGER_HPP

#include <vector>
#include <entt/entity/fwd.hpp>
#include <entt/entity/sparse_set.hpp>

namespace edyn {

class registry_operation_builder;

class island_manager {
    void init_new_nodes_and_edges();
    entt::entity create_island();
    void insert_to_island(entt::entity island_entity,
                          const std::vector<entt::entity> &nodes,
                          const std::vector<entt::entity> &edges);
    void merge_islands(const std::vector<entt::entity> &island_entities,
                       const std::vector<entt::entity> &new_nodes,
                       const std::vector<entt::entity> &new_edges);
    void split_islands();

public:
    island_manager(entt::registry &registry, registry_operation_builder *op_builder = nullptr);
    void on_construct_graph_node(entt::registry &, entt::entity);
    void on_construct_graph_edge(entt::registry &, entt::entity);
    void on_destroy_graph_node(entt::registry &, entt::entity);
    void on_destroy_graph_edge(entt::registry &, entt::entity);
    void on_destroy_island_resident(entt::registry &, entt::entity);
    void on_destroy_multi_island_resident(entt::registry &, entt::entity);

    void wake_up_island(entt::entity island_entity);
    void put_to_sleep(entt::entity island_entity);
    bool could_go_to_sleep(entt::entity island_entity) const;

private:
    entt::registry *m_registry;
    registry_operation_builder *m_op_builder;
    std::vector<entt::entity> m_new_graph_nodes;
    std::vector<entt::entity> m_new_graph_edges;
    entt::sparse_set m_islands_to_split;
};

}

#endif // EDYN_PARALLEL_ISLAND_MANAGER_HPP
