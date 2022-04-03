#ifndef EDYN_PARALLEL_ENTITY_GRAPH_HPP
#define EDYN_PARALLEL_ENTITY_GRAPH_HPP

#include <vector>
#include <cstdint>
#include <limits>
#include <entt/entity/fwd.hpp>
#include <entt/entity/entity.hpp>
#include "edyn/config/config.h"
#include "edyn/util/entity_pair.hpp"

namespace edyn {

/**
 * A non-directed, unweighted graph where multiple edges can exist between
 * the same pair of nodes (i.e. multigraph) and each node and edge hold an
 * `entt::entity` payload.
 */
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
    void remove_all_edges(index_type node_index);
    entt::entity edge_entity(index_type edge_index) const;
    bool has_adjacency(index_type node_index0, index_type node_index1) const;
    entity_pair edge_node_entities(index_type edge_index) const;

    bool is_connecting_node(index_type node_index) const;

    /**
     * @brief Calculate whether this graph contains a single connected component.
     * @return Whether this graph is a single connected component.
     */
    bool is_single_connected_component();

    /**
     * @brief Visit neighboring nodes of a node.
     * @tparam Func Visitor function type.
     * @param node_index Index of node to be visited.
     * @param func Visitor function with signature `void(entt::entity)`.
     */
    template<typename Func>
    void visit_neighbors(index_type node_index, Func func) const;

    /**
     * @brief Visit all edges between two nodes.
     * @tparam Func Visitor function type.
     * @param node_index0 Index of first node.
     * @param node_index1 Index of second node.
     * @param func Vistor function with signature `void(index_type)`.
     */
    template<typename Func>
    void visit_edges(index_type node_index0, index_type node_index1, Func func) const;

    /**
     * @brief Visit all edges originating at a node.
     * @tparam Func Visitor function type.
     * @param node_index0 Index of node.
     * @param func Vistor function with signature `void(index_type)`.
     */
    template<typename Func>
    void visit_edges(index_type node_index, Func func) const;

    /**
     * @brief Visits nodes and edges that can be reached from the provided range
     * of nodes.
     * @tparam It Node container iterator type.
     * @tparam VisitNodeFunc Type of node visitor function.
     * @tparam VisitEdgeFunc Type of edge visitor function.
     * @tparam ShouldFunc Type of function that decides whether to visit a node.
     * @tparam ComponentFunc Type of function that's called for every connected
     * component found.
     * @param first An iterator to the first element of a range of node indices.
     * @param last An iterator past the last element of a range of node indices.
     * @param visitNodeFunc Function called for each node being visited.
     * @param visitEdgeFunc Function called for each edge being visited.
     * @param shouldFunc Function that returns whether to proceed visitng node.
     * @param componentFunc Function called for each connected component that is
     * formed.
     */
    template<typename It, typename VisitNodeFunc,
             typename VisitEdgeFunc, typename ShouldFunc,
             typename ComponentFunc>
    void reach(It first, It last, VisitNodeFunc visitNodeFunc,
               VisitEdgeFunc visitEdgeFunc, ShouldFunc shouldFunc,
               ComponentFunc componentFunc);

    using connected_components_t = std::vector<connected_component>;

    /**
     * Calculates and returns all connected components of this graph.
     * Non-connecting nodes are not walked through and can be present in
     * multiple connected components.
     * @return The connected components.
     */
    connected_components_t connected_components();

    /**
     * @brief Traverses nodes starting at the given node, ignoring
     * non-connecting nodes.
     * @tparam Func Function type with signature `void(index_type)`.
     * @param start_nodex_index Index of node where traversal starts.
     * @param func Function called for each node.
     */
    template<typename Func>
    void traverse_connecting_nodes(index_type start_node_index, Func func);

    void optimize_if_needed();

private:
    std::vector<node> m_nodes;
    std::vector<edge> m_edges;
    std::vector<adjacency> m_adjacencies;

    std::vector<bool> m_visited;
    std::vector<bool> m_visited_edges;

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
        EDYN_ASSERT(adj.next != adj_index);
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
                func(edge_index);
                auto &edge = m_edges[edge_index];
                EDYN_ASSERT(edge.next != edge_index);
                edge_index = edge.next;
            }
            break;
        }
        adj_index = adj.next;
    }
}

template<typename Func>
void entity_graph::visit_edges(index_type node_index, Func func) const {
    EDYN_ASSERT(node_index < m_nodes.size());

    auto adj_index = m_nodes[node_index].adjacency_index;

    while (adj_index != null_index) {
        auto &adj = m_adjacencies[adj_index];
        auto edge_index = adj.edge_index;

        while (edge_index != null_index) {
            func(edge_index);
            auto &edge = m_edges[edge_index];
            EDYN_ASSERT(edge.next != edge_index);
            edge_index = edge.next;
        }

        adj_index = adj.next;
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
    m_visited_edges.assign(m_edges.size(), false);

    std::vector<index_type> non_connecting_indices;
    std::vector<index_type> to_visit;

    for (auto it = first; it != last; ++it) {
        auto start_node_index = *it;
        // All provided nodes are expected to be connecting.
        EDYN_ASSERT(!m_nodes[start_node_index].non_connecting);

        if (m_visited[start_node_index]) {
            continue;
        }

        if (!shouldFunc(start_node_index)) {
            continue;
        }

        to_visit.push_back(start_node_index);

        // Visit nodes reachable from the initial node inserted into `to_visit`.
        while (!to_visit.empty()) {
            auto node_index = to_visit.back();
            to_visit.pop_back();

            m_visited[node_index] = true;

            const auto &node = m_nodes[node_index];
            EDYN_ASSERT(node.entity != entt::null);
            visitNodeFunc(node.entity);

            // Stop here for non-connecting nodes. Do not visit edges.
            if (node.non_connecting) {
                non_connecting_indices.push_back(node_index);
                continue;
            }

            // Visit all edges in all adjacencies.
            auto adj_index = node.adjacency_index;

            while (adj_index != null_index) {
                auto &adj = m_adjacencies[adj_index];
                auto edge_index = adj.edge_index;

                while (edge_index != null_index) {
                    auto &edge = m_edges[edge_index];

                    if (!m_visited_edges[edge_index]) {
                        EDYN_ASSERT(edge.entity != entt::null);
                        visitEdgeFunc(edge.entity);
                        m_visited_edges[edge_index] = true;
                    }

                    edge_index = edge.next;
                }

                // Perhaps visit neighboring node and its edges next.
                auto neighbor_index = adj.node_index;

                if (!m_visited[neighbor_index] && shouldFunc(neighbor_index)) {
                    to_visit.emplace_back(neighbor_index);
                    // Set as visited to avoid adding it to `to_visit` more than once.
                    m_visited[neighbor_index] = true;
                }

                adj_index = adj.next;
            }
        }

        // Finished one connected component.
        componentFunc();

        // Set non-connecting nodes as not visited so they'll be visited again
        // for the next connected components. Non-connecting nodes are shared
        // among connected components.
        for (auto node_index : non_connecting_indices) {
            m_visited[node_index] = false;
        }

        non_connecting_indices.clear();
        to_visit.clear();
    }
}

template<typename Func>
void entity_graph::traverse_connecting_nodes(index_type start_node_index, Func func) {
    m_visited.assign(m_nodes.size(), false);

    std::vector<index_type> to_visit;
    to_visit.push_back(start_node_index);

    while (!to_visit.empty()) {
        auto node_index = to_visit.back();
        to_visit.pop_back();

        m_visited[node_index] = true;
        const auto &node = m_nodes[node_index];
        EDYN_ASSERT(node.entity != entt::null);

        // Ignore non-connecting nodes.
        if (node.non_connecting) {
            continue;
        }

        func(node_index);

        // Add neighbors to be visited.
        auto adj_index = node.adjacency_index;

        while (adj_index != null_index) {
            auto &adj = m_adjacencies[adj_index];
            auto neighbor_index = adj.node_index;

            if (!m_visited[neighbor_index]) {
                // Insert to beginning for a breadth-first traversal.
                to_visit.insert(to_visit.begin(), neighbor_index);
                // Set as visited to avoid adding it to `to_visit` more than once.
                m_visited[neighbor_index] = true;
            }

            adj_index = adj.next;
        }
    }
}

}

#endif // EDYN_PARALLEL_ENTITY_GRAPH_HPP
