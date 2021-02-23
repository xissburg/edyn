#ifndef EDYN_PARALLEL_ENTITY_GRAPH_HPP
#define EDYN_PARALLEL_ENTITY_GRAPH_HPP

#include <vector>
#include <cstdint>
#include <limits>
#include <entt/fwd.hpp>
#include <entt/entity/entity.hpp>
#include "edyn/config/config.h"
#include "edyn/util/entity_pair.hpp"

namespace edyn {

class entity_graph final {
public:
    using index_type = size_t;
    constexpr static index_type null_index = std::numeric_limits<index_type>::max();

    struct connected_component {
        std::vector<entt::entity> nodes;
        std::vector<entt::entity> edges;
    };

private:
    struct node {
        entt::entity entity;
        bool non_connecting;
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
    index_type create_adjacency(index_type destination_node_index, index_type edge_index);
    void remove_adjacency_edge(index_type source_node_index, index_type adj_index, index_type edge_index);
    void remove_adjacency(index_type source_node_index, index_type adj_index);

    double efficiency() const;
    void optimize();

public:
    index_type insert_node(entt::entity entity, bool non_connecting = false);
    void remove_node(index_type node_index);
    entt::entity node_entity(index_type node_index) const;

    index_type insert_edge(entt::entity entity, index_type node_index0, index_type node_index1);
    void remove_edge(index_type edge_index);
    entt::entity edge_entity(index_type edge_index) const;
    bool has_edge(index_type node_index0, index_type node_index1) const;
    entity_pair edge_node_entities(index_type edge_index) const;

    bool is_single_connected_component();

    template<typename Func>
    void visit_neighbors(index_type node_index, Func func) const;

    template<typename Func>
    void visit_edges(index_type node_index0, index_type node_index1, Func func) const;

    template<typename Func>
    void visit_edges(index_type adj_index, Func func) const;

    template<typename It, typename VisitNodeFunc, 
             typename VisitEdgeFunc, typename ShouldFunc, 
             typename ComponentFunc>
    void reach(It first, It last, VisitNodeFunc visitNodeFunc, 
               VisitEdgeFunc visitEdgeFunc, ShouldFunc shouldFunc, 
               ComponentFunc componentFunc);
    
    using connected_components_t = std::vector<connected_component>;

    template<typename It>
    connected_components_t connected_components(It first, It last);

    void optimize_if_needed();

private:
    std::vector<node> m_nodes;
    std::vector<edge> m_edges;
    std::vector<adjacency> m_adjacencies;

    std::vector<bool> m_visited;

    size_t m_node_count;
    size_t m_edge_count;

    size_t m_nodes_free_list {null_index};
    size_t m_edges_free_list {null_index};
    size_t m_adjacencies_free_list {null_index};
};

template<typename Func>
void entity_graph::visit_neighbors(index_type node_index, Func func) const {
    EDYN_ASSERT(node_index < m_nodes.size());
    EDYN_ASSERT(m_nodes[node_index].entity != entt::null);

    auto adj_index = m_nodes[node_index].adjacency_index;

    while (adj_index != null_index) {
        auto &adj = m_adjacencies[adj_index];
        auto &neighbor = m_nodes[adj.node_index];
        EDYN_ASSERT(neighbor.entity != entt::null);
        func(neighbor.entity);
        adj_index = adj.next;
    }
}

template<typename Func>
void entity_graph::visit_edges(index_type node_index0, index_type node_index1, Func func) const {
    EDYN_ASSERT(node_index0 < m_nodes.size());
    EDYN_ASSERT(node_index1 < m_nodes.size());

    auto adj_index = m_nodes[node_index0].adjacency_index;
    while (adj_index != null_index) {
        auto &adj = m_adjacencies[adj_index];
        if (adj.node_index == node_index1) {
            auto edge_index = adj.edge_index;
            while (edge_index != null_index) {
                auto &edge = m_edges[edge_index];
                EDYN_ASSERT(edge.entity != entt::null);
                func(edge.entity);
                edge_index = edge.next;
            }
            break;
        }
        adj_index = adj.next;
    }
}

template<typename Func>
void entity_graph::visit_edges(index_type adj_index, Func func) const {
    EDYN_ASSERT(adj_index < m_adjacencies.size());
    EDYN_ASSERT(m_adjacencies[adj_index].edge_index != null_index);

    auto edge_index = m_adjacencies[adj_index].edge_index;

    while (edge_index != null_index) {
        auto &edge = m_edges[edge_index];
        EDYN_ASSERT(edge.entity != entt::null);
        func(edge.entity);
        edge_index = edge.next;
    }
}

template<typename It, typename VisitNodeFunc, 
         typename VisitEdgeFunc, typename ShouldFunc, 
         typename ComponentFunc>
void entity_graph::reach(It first, It last, VisitNodeFunc visitNodeFunc, 
                         VisitEdgeFunc visitEdgeFunc, ShouldFunc shouldFunc, 
                         ComponentFunc componentFunc) {
    EDYN_ASSERT(std::distance(first, last) > 0);

    m_visited.assign(m_nodes.size(), false);

    // Pairs of node index and adjacency index.
    std::vector<std::pair<index_type, index_type>> to_visit;
    to_visit.emplace_back(*first, null_index);

    while (true) {
        // Visit nodes reachable from the initial node inserted into `to_visit`.
        while (!to_visit.empty()) {
            auto [node_index, adj_index] = to_visit.back();
            to_visit.pop_back();

            m_visited[node_index] = true;

            const auto &node = m_nodes[node_index];
            EDYN_ASSERT(node.entity != entt::null);
            visitNodeFunc(node.entity);

            if (adj_index != null_index) {
                auto edge_index = m_adjacencies[adj_index].edge_index;
                while (edge_index != null_index) {
                    const auto &edge = m_edges[edge_index];
                    EDYN_ASSERT(edge.entity != entt::null);
                    visitEdgeFunc(edge.entity);
                    edge_index = edge.next;
                }
            }

            adj_index = node.adjacency_index;

            while (adj_index != null_index) {
                auto neighbor_index = m_adjacencies[adj_index].node_index;

                if (!m_visited[neighbor_index] && shouldFunc(neighbor_index)) {
                    to_visit.emplace_back(neighbor_index, adj_index);
                }

                adj_index = m_adjacencies[adj_index].next;
            }
        }

        componentFunc();

        // Look for a node in the list that has not yet been visited.
        for (auto it = first; it != last; ++it) {
            auto node_index = *it;
            if (m_nodes[node_index].entity != entt::null && 
                !m_visited[node_index]) {
                to_visit.emplace_back(node_index, null_index);
                break;
            }
        }

        // No more nodes left to visit.
        if (to_visit.empty()) {
            break;
        }
    }
}


template<typename It>
entity_graph::connected_components_t entity_graph::connected_components(It first, It last) {
    EDYN_ASSERT(std::distance(first, last) > 0);

    auto components = entity_graph::connected_components_t{};
    m_visited.assign(m_nodes.size(), false);

    // Pairs of node index and adjacency index.
    std::vector<std::pair<index_type, index_type>> to_visit;

    for (auto it = first; it != last; ++it) {
        if (!m_nodes[*it].non_connecting) {
            to_visit.emplace_back(*it, null_index);
            break;
        }
    }

    std::vector<index_type> non_connecting_indices;

    while (true) {
        auto &connected = components.emplace_back();

        // Visit nodes reachable from the initial node inserted into `to_visit`.
        while (!to_visit.empty()) {
            auto [node_index, adj_index] = to_visit.back();
            to_visit.pop_back();

            m_visited[node_index] = true;

            auto &node = m_nodes[node_index];
            connected.nodes.push_back(node.entity);

            if (adj_index != null_index) {
                visit_edges(adj_index, [&connected] (entt::entity edge_entity) {
                    connected.edges.push_back(edge_entity);
                });
            }

            if (node.non_connecting) {
                non_connecting_indices.push_back(node_index);
                continue;
            }

            adj_index = node.adjacency_index;

            while (adj_index != null_index) {
                auto neighbor_index = m_adjacencies[adj_index].node_index;

                if (!m_visited[neighbor_index]) {
                    to_visit.emplace_back(neighbor_index, adj_index);
                }

                adj_index = m_adjacencies[adj_index].next;
            }
        }

        for (auto node_index : non_connecting_indices) {
            m_visited[node_index] = false;
        }
        non_connecting_indices.clear();

        // Look for a node in the list that has not yet been visited.
        for (auto it = first; it != last; ++it) {
            auto node_index = *it;
            EDYN_ASSERT(m_nodes[node_index].entity != entt::null);
            if (!m_visited[node_index] && !m_nodes[node_index].non_connecting) {
                to_visit.emplace_back(node_index, null_index);
                break;
            }
        }

        // No more nodes left to visit.
        if (to_visit.empty()) {
            break;
        }
    }

    return components;
}


}

#endif // EDYN_PARALLEL_ENTITY_GRAPH_HPP