#ifndef EDYN_DYNAMICS_SOLVER_HPP
#define EDYN_DYNAMICS_SOLVER_HPP

#include <vector>
#include <memory>
#include <cstdint>
#include <unordered_map>
#include <entt/fwd.hpp>
#include <entt/entity/registry.hpp>
#include "edyn/math/scalar.hpp"
#include "edyn/comp/constraint_group.hpp"

namespace edyn {

struct job;
struct solver_context;
struct island_node;
struct constraint;

class solver {
    struct state {
        size_t iteration {0};
        scalar dt;
    };

    void init_new_nodes();
    void init_new_edges();
    bool is_constraint_graph_unbalanced() const;
    void partite_constraint_graph();
    void sort_constraint_rows();
    void prepare_constraint_graph();
    void run_async_iteration();
    void dispatch_solver_job();

    template<typename It>
    void refresh_edges(It first, It last);

    using group_node_view_t = entt::basic_view<entt::entity, entt::exclude_t<>, constraint_group, island_node>; 
    using constraint_view_t = entt::basic_view<entt::entity, entt::exclude_t<>, constraint>; 
    void refresh_edge(entt::entity, group_node_view_t &, constraint_view_t &);

public:
    solver(entt::registry &);
    ~solver();

    bool parallelizable() const;
    void update(scalar dt);
    void start_async_update(scalar dt, const job &completion);
    bool continue_async_update();
    void finish_async_update();

    void on_construct_constraint_graph_node(entt::registry &, entt::entity);
    void on_destroy_constraint_graph_node(entt::registry &, entt::entity);
    void on_construct_constraint_graph_edge(entt::registry &, entt::entity);
    void on_destroy_constraint_graph_edge(entt::registry &, entt::entity);
    void on_change_constraint_rows(entt::registry &, entt::entity);

    uint32_t iterations {10};

private:
    entt::registry *m_registry;
    bool m_constraints_changed;
    bool m_constraint_rows_changed;
    bool m_nodes_destroyed;
    state m_state;
    std::vector<entt::entity> m_new_nodes;
    std::vector<entt::entity> m_new_edges;
    std::unordered_map<constraint_group::value_t, size_t> m_group_sizes;
    std::unique_ptr<solver_context> m_context;
    size_t m_num_constraint_groups;
    static const size_t m_max_group_size {128};
};

template<typename It>
void solver::refresh_edges(It first, It last) {
    auto group_node_view = m_registry->view<constraint_group, island_node>();
    auto constraint_view = m_registry->view<constraint>();
    
    for (auto it = first; it != last; ++it) {
        auto edge_entity = *it;
        refresh_edge(edge_entity, group_node_view, constraint_view);
    }
}

}

#endif // EDYN_DYNAMICS_SOLVER_HPP