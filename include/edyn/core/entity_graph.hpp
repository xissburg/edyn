#ifndef EDYN_CORE_ENTITY_GRAPH_HPP
#define EDYN_CORE_ENTITY_GRAPH_HPP

#include <array>
#include <vector>
#include <cstdint>
#include <limits>
#include <type_traits>
#include <entt/entity/fwd.hpp>
#include <entt/entity/entity.hpp>
#include "edyn/config/config.h"
#include "edyn/core/entity_pair.hpp"

namespace edyn {

namespace internal {
    struct no_edge_visits {};
}

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
    std::array<index_type, 2> edge_node_indices(index_type edge_index) const;
    entity_pair edge_node_entities(index_type edge_index) const;

    bool is_connecting_node(index_type node_index) const;
    void set_connecting_node(index_type node_index, bool connecting);

    /**
     * @brief Calculate whether this graph contains a single connected component.
     * @return Whether this graph is a single connected component.
     */
    bool is_single_connected_component() const;

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
     * @param func Visitor function with signature `void(index_type)` or
     * `bool(index_type)`. The latter can return false to abort the visit.
     */
    template<typename Func>
    void visit_edges(index_type node_index0, index_type node_index1, Func func) const;

    /**
     * @brief Visit all edges originating at a node.
     * @tparam Func Visitor function type.
     * @param node_index0 Index of node.
     * @param func Visitor function with signature `void(index_type)` or
     * `bool(index_type)`. The latter can return false to abort the visit.
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
     * @param visit_node_func Function called for each node being visited.
     * @param visit_edge_func Function called for each edge being visited.
     * @param should_func Function that returns whether to proceed visiting node.
     * @param component_func Function called for each connected component that is
     * formed.
     */
    template<typename It, typename VisitNodeFunc,
             typename VisitEdgeFunc, typename ShouldFunc,
             typename ComponentFunc>
    void reach(It first, It last, VisitNodeFunc visit_node_func,
               VisitEdgeFunc visit_edge_func, ShouldFunc should_func,
               ComponentFunc component_func) const;

    using connected_components_t = std::vector<connected_component>;

    /**
     * Calculates and returns all connected components of this graph.
     * Non-connecting nodes are not walked through and can be present in
     * multiple connected components.
     * @return The connected components.
     */
    connected_components_t connected_components() const;

    /**
     * @brief Traverses nodes starting at the given node. Neighbors of
     * non-connecting nodes aren't visited.
     * @tparam VisitNodeFunc Function type with signature `void(index_type)`
     * @tparam VisitEdgeFunc Function type with signature `void(index_type)`
     * @param start_node_index Index of node where traversal starts.
     * @param visit_node_func Function called for each node.
     * @param visit_edge_func Optional function called for each edge.
     */
    template<typename VisitNodeFunc, typename VisitEdgeFunc = internal::no_edge_visits>
    void traverse(index_type start_node_index,
                  VisitNodeFunc visit_node_func,
                  VisitEdgeFunc visit_edge_func = {}) const;

    void optimize_if_needed();

    void clear();

private:
    std::vector<node> m_nodes;
    std::vector<edge> m_edges;
    std::vector<adjacency> m_adjacencies;

    size_t m_node_count {};
    size_t m_edge_count {};

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
        if constexpr(std::is_invocable_r_v<bool, Func, entt::entity>) {
            if (!func(neighbor.entity)) {
                return;
            }
        } else {
            func(neighbor.entity);
        }
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
                if constexpr(std::is_invocable_r_v<bool, Func, index_type>) {
                    if (!func(edge_index)) {
                        return;
                    }
                } else {
                    func(edge_index);
                }
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
        auto next_adj_index = adj.next;
        auto edge_index = adj.edge_index;

        while (edge_index != null_index) {
            auto &edge = m_edges[edge_index];
            EDYN_ASSERT(edge.next != edge_index);
            auto next_edge_index = edge.next;
            if constexpr(std::is_invocable_r_v<bool, Func, index_type>) {
                if (!func(edge_index)) {
                    return;
                }
            } else {
                func(edge_index);
            }
            edge_index = next_edge_index;
        }

        adj_index = next_adj_index;
    }
}

template<typename It, typename VisitNodeFunc,
         typename VisitEdgeFunc, typename ShouldFunc,
         typename ComponentFunc>
void entity_graph::reach(It first, It last, VisitNodeFunc visit_node_func,
                         VisitEdgeFunc visit_edge_func, ShouldFunc should_func,
                         ComponentFunc component_func) const {
    EDYN_ASSERT(std::distance(first, last) > 0);

    std::vector<bool> visited;
    std::vector<bool> visited_edges;
    visited.assign(m_nodes.size(), false);
    visited_edges.assign(m_edges.size(), false);

    std::vector<index_type> non_connecting_indices;
    std::vector<index_type> to_visit;

    for (auto it = first; it != last; ++it) {
        auto start_node_index = *it;
        // All provided nodes are expected to be connecting.
        EDYN_ASSERT(!m_nodes[start_node_index].non_connecting);

        if (visited[start_node_index]) {
            continue;
        }

        if (!should_func(start_node_index)) {
            continue;
        }

        to_visit.push_back(start_node_index);

        // Visit nodes reachable from the initial node inserted into `to_visit`.
        while (!to_visit.empty()) {
            auto node_index = to_visit.back();
            to_visit.pop_back();

            visited[node_index] = true;

            const auto &node = m_nodes[node_index];
            EDYN_ASSERT(node.entity != entt::null);
            visit_node_func(node.entity);

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

                    if (!visited_edges[edge_index]) {
                        EDYN_ASSERT(edge.entity != entt::null);
                        visit_edge_func(edge.entity);
                        visited_edges[edge_index] = true;
                    }

                    edge_index = edge.next;
                }

                // Perhaps visit neighboring node and its edges next.
                auto neighbor_index = adj.node_index;

                if (!visited[neighbor_index] && should_func(neighbor_index)) {
                    to_visit.emplace_back(neighbor_index);
                    // Set as visited to avoid adding it to `to_visit` more than once.
                    visited[neighbor_index] = true;
                }

                adj_index = adj.next;
            }
        }

        // Finished one connected component.
        component_func();

        // Set non-connecting nodes as not visited so they'll be visited again
        // for the next connected components. Non-connecting nodes are shared
        // among connected components.
        for (auto node_index : non_connecting_indices) {
            visited[node_index] = false;
        }

        non_connecting_indices.clear();
        to_visit.clear();
    }
}

template<typename VisitNodeFunc, typename VisitEdgeFunc>
void entity_graph::traverse(index_type start_node_index,
                            VisitNodeFunc visit_node_func,
                            VisitEdgeFunc visit_edge_func) const {
    std::vector<bool> visited;
    std::vector<bool> visited_edges;
    visited.assign(m_nodes.size(), false);

    constexpr auto should_visit_edges = !std::is_same_v<VisitEdgeFunc, internal::no_edge_visits>;

    if constexpr(should_visit_edges) {
        visited_edges.assign(m_edges.size(), false);
    }

    std::vector<index_type> to_visit;
    to_visit.push_back(start_node_index);

    while (!to_visit.empty()) {
        auto node_index = to_visit.back();
        to_visit.pop_back();

        visited[node_index] = true;
        const auto &node = m_nodes[node_index];
        EDYN_ASSERT(node.entity != entt::null);

        visit_node_func(node_index);

        // Do not visit neighbors of non-connecting nodes.
        if (node.non_connecting) {
            continue;
        }

        // Add neighbors to be visited.
        auto adj_index = node.adjacency_index;

        while (adj_index != null_index) {
            auto &adj = m_adjacencies[adj_index];

            if constexpr(should_visit_edges) {
                auto edge_index = adj.edge_index;

                while (edge_index != null_index) {
                    auto &edge = m_edges[edge_index];

                    if (!visited_edges[edge_index]) {
                        EDYN_ASSERT(edge.entity != entt::null);
                        visit_edge_func(edge_index);
                        visited_edges[edge_index] = true;
                    }

                    edge_index = edge.next;
                }
            }

            auto neighbor_index = adj.node_index;

            if (!visited[neighbor_index]) {
                // Insert to beginning for a breadth-first traversal.
                to_visit.insert(to_visit.begin(), neighbor_index);
                // Set as visited to avoid adding it to `to_visit` more than once.
                visited[neighbor_index] = true;
            }

            adj_index = adj.next;
        }
    }
}

}

#endif // EDYN_CORE_ENTITY_GRAPH_HPP
