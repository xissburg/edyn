#ifndef EDYN_PARALLEL_ENTITY_GRAPH_HPP
#define EDYN_PARALLEL_ENTITY_GRAPH_HPP

#include <vector>
#include <cstdint>
#include <limits>
#include <entt/fwd.hpp>
#include <entt/entity/entity.hpp>
#include "edyn/config/config.h"

namespace edyn {

class entity_graph final {
public:
    using index_type = size_t;
    constexpr static index_type null_index = std::numeric_limits<index_type>::max();

private:
    struct node {
        entt::entity entity;
        index_type adjacency_index;
        index_type next;
    };

    struct edge {
        entt::entity entity;
        index_type node_index0, node_index1;
        index_type adj_index0, adj_index1;
        index_type next;
    };

    struct adjacency {
        index_type node_index;
        index_type edge_index;
        index_type next;
    };

    void insert_adjacency(index_type node_index0, index_type node_index1, index_type edge_index);
    index_type insert_adjacency_one_way(index_type node_index0, index_type node_index1, index_type edge_index);
    index_type create_adjacency(index_type node_index, index_type edge_index);
    void remove_adjacency_edge(index_type adj_index, index_type edge_index);
    void remove_adjacency(index_type adj_index);

    double efficiency() const;
    void optimize();

public:
    index_type insert_node(entt::entity entity);
    void remove_node(index_type node_index);
    entt::entity node_entity(index_type node_index) const;

    index_type insert_edge(entt::entity entity, index_type node_index0, index_type node_index1);
    void remove_edge(index_type edge_index);
    entt::entity edge_entity(index_type edge_index) const;
    bool has_edge(index_type node_index0, index_type node_index1) const;

    bool is_single_connected_component() const;

    template<typename It, typename VisitFunc, typename ShouldFunc, typename ComponentFunc>
    void reach(It first, It last, VisitFunc visitFunc, ShouldFunc shouldFunc, ComponentFunc componentFunc) const;

    void optimize_if_needed();

private:
    std::vector<node> m_nodes;
    std::vector<edge> m_edges;
    std::vector<adjacency> m_adjacencies;

    size_t m_node_count;
    size_t m_edge_count;

    size_t m_nodes_free_list {null_index};
    size_t m_edges_free_list;
    size_t m_adjacencies_free_list;
};

template<typename It, typename VisitFunc, typename ShouldFunc, typename ComponentFunc>
void entity_graph::reach(It first, It last, VisitFunc visitFunc, ShouldFunc shouldFunc, ComponentFunc componentFunc) const {
    index_type min_index = std::numeric_limits<index_type>::max();
    index_type max_index = std::numeric_limits<index_type>::min();

    for (auto it = first; it != last; ++it) {
        if (*it < min_index) {
            min_index = *it;
        }
        if (*it > max_index) {
            max_index = *it;
        }
    }

    EDYN_ASSERT(min_index < m_nodes.size());
    EDYN_ASSERT(max_index < m_nodes.size());

    auto dist = max_index - min_index + 1;
    std::vector<bool> visited(dist, false);
    std::vector<std::pair<index_type, index_type>> to_visit;
    to_visit.emplace_back(*first, null_index);

    while (true) {
        // Visit nodes reachable from the initial node inserted into `to_visit`.
        while (!to_visit.empty()) {
            auto [node_index, edge_index] = to_visit.back();
            to_visit.pop_back();

            visited[node_index - min_index] = true;

            visitFunc(node_index, edge_index);

            auto adj = m_nodes[node_index].adjacency_index;

            while (adj != null_index) {
                auto neighbor_index = m_adjacencies[adj].node_index;
                adj = m_adjacencies[adj].next;

                if (shouldFunc(neighbor_index) &&
                    neighbor_index >= min_index && 
                    neighbor_index <= max_index && 
                    !visited[neighbor_index - min_index]) {
                    auto edge_index =  m_adjacencies[adj].edge_index;
                    to_visit.emplace_back(neighbor_index, edge_index);
                }
            }
        }

        componentFunc();

        // Look for a node in the list that has not yet been visited.
        for (auto it = first; it != last; ++it) {
            auto node_index = *it;
            if (m_nodes[node_index].entity != entt::null && 
                !visited[node_index - min_index]) {
                to_visit.push_back(node_index);
                break;
            }
        }

        // No more nodes left to visit.
        if (to_visit.empty()) {
            break;
        }
    }
}

}

#endif // EDYN_PARALLEL_ENTITY_GRAPH_HPP