#include "edyn/core/entity_graph.hpp"
#include "edyn/config/config.h"

namespace edyn {

static constexpr size_t allocation_size = 16;

entity_graph::index_type entity_graph::insert_node(entt::entity entity, bool non_connecting) {
    EDYN_ASSERT(entity != entt::null);

    if (m_nodes_free_list == null_index) {
        m_nodes_free_list = m_nodes.size();
        m_nodes.resize(m_nodes.size() + allocation_size);

        for (auto i = m_nodes_free_list; i < m_nodes.size(); ++i) {
            auto &node = m_nodes[i];
            node.next = i + 1;
            node.adjacency_index = null_index;
            node.entity = entt::null;
        }

        m_nodes.back().next = null_index;
    }

    auto index = m_nodes_free_list;
    auto &node = m_nodes[index];
    m_nodes_free_list = node.next;
    node.entity = entity;
    node.non_connecting = non_connecting;
    ++m_node_count;

    return index;
}

void entity_graph::remove_node(index_type node_index) {
    EDYN_ASSERT(node_index < m_nodes.size());
    EDYN_ASSERT(m_nodes[node_index].entity != entt::null);

    // Node must not have any edges.
    EDYN_ASSERT(m_nodes[node_index].adjacency_index == null_index);

    m_nodes[node_index].entity = entt::null;
    m_nodes[node_index].next = m_nodes_free_list;
    m_nodes_free_list = node_index;
    --m_node_count;
}

entt::entity entity_graph::node_entity(index_type node_index) const {
    EDYN_ASSERT(node_index < m_nodes.size());
    EDYN_ASSERT(m_nodes[node_index].entity != entt::null);
    return m_nodes[node_index].entity;
}

entity_graph::index_type entity_graph::insert_edge(entt::entity entity, index_type node_index0, index_type node_index1) {
    EDYN_ASSERT(entity != entt::null);
    EDYN_ASSERT(node_index0 < m_nodes.size() && node_index1 < m_nodes.size());
    EDYN_ASSERT(m_nodes[node_index0].entity != entt::null);
    EDYN_ASSERT(m_nodes[node_index1].entity != entt::null);

    if (m_edges_free_list == null_index) {
        m_edges_free_list = m_edges.size();
        m_edges.resize(m_edges.size() + allocation_size);

        for (auto i = m_edges_free_list; i < m_edges.size(); ++i) {
            auto &edge = m_edges[i];
            edge.next = i + 1;
            edge.node_index0 = null_index;
            edge.node_index1 = null_index;
            edge.adj_index0 = null_index;
            edge.adj_index1 = null_index;
            edge.entity = entt::null;
        }

        m_edges.back().next = null_index;
    }

    auto edge_index = m_edges_free_list;
    auto &edge = m_edges[edge_index];
    m_edges_free_list = edge.next;
    edge.entity = entity;
    edge.node_index0 = node_index0;
    edge.node_index1 = node_index1;
    edge.next = null_index;

    // Look for an existing adjacency.
    auto adj_index = m_nodes[node_index0].adjacency_index;

    while (adj_index != null_index) {
        if (m_adjacencies[adj_index].node_index == node_index1) {
            break; // Adjacency between node_index0 and node_index1 exists.
        }
        adj_index = m_adjacencies[adj_index].next;
    }

    if (adj_index != null_index) {
        edge.adj_index0 = adj_index;

        // Get edge from adjacency and add this new edge to the beginning of
        // the edge list.
        auto adj_edge_index = m_adjacencies[adj_index].edge_index;
        EDYN_ASSERT(adj_edge_index != null_index);
        edge.next = adj_edge_index;
        m_adjacencies[adj_index].edge_index = edge_index;

        // Repeat for the corresponding adjacency of node_index1.
        adj_index = m_nodes[node_index1].adjacency_index;

        while (adj_index != null_index) {
            if (m_adjacencies[adj_index].node_index == node_index0) {
                break;
            }
            adj_index = m_adjacencies[adj_index].next;
        }

        // Corresponding adjacency must exist.
        EDYN_ASSERT(adj_index != null_index);
        // Corresponding adjacency must be pointing to the same edge.
        EDYN_ASSERT(m_adjacencies[adj_index].edge_index == adj_edge_index);
        m_adjacencies[adj_index].edge_index = edge_index;
        edge.adj_index1 = adj_index;
    } else {
        insert_adjacency(node_index0, node_index1, edge_index);
    }

    ++m_edge_count;

    return edge_index;
}

void entity_graph::remove_edge(index_type edge_index) {
    EDYN_ASSERT(edge_index < m_edges.size());
    EDYN_ASSERT(m_edges[edge_index].entity != entt::null);

    auto &edge = m_edges[edge_index];
    auto &node0 = m_nodes[edge.node_index0];

    auto adj_index0 = node0.adjacency_index;
    while (m_adjacencies[adj_index0].node_index != edge.node_index1) {
        EDYN_ASSERT(adj_index0 != null_index);
        adj_index0 = m_adjacencies[adj_index0].next;
    }

    auto first_edge_index = m_adjacencies[adj_index0].edge_index;

    if (edge.node_index1 != edge.node_index0) {
        auto &node1 = m_nodes[edge.node_index1];

        auto adj_index1 = node1.adjacency_index;
        while (m_adjacencies[adj_index1].node_index != edge.node_index0) {
            EDYN_ASSERT(adj_index1 != null_index);
            adj_index1 = m_adjacencies[adj_index1].next;
        }

        EDYN_ASSERT(m_adjacencies[adj_index0].edge_index == m_adjacencies[adj_index1].edge_index);
        remove_adjacency_edge(edge.node_index1, adj_index1, edge_index);
    }

    remove_adjacency_edge(edge.node_index0, adj_index0, edge_index);

    if (edge_index != first_edge_index) {
        // Find edge before the one being removed.
        auto prev_edge_index = first_edge_index;

        while (m_edges[prev_edge_index].next != edge_index) {
            EDYN_ASSERT(m_edges[prev_edge_index].next != null_index);
            prev_edge_index = m_edges[prev_edge_index].next;
        }

        m_edges[prev_edge_index].next = edge.next;
    }

    edge.entity = entt::null;
    edge.next = m_edges_free_list;
    m_edges_free_list = edge_index;
    --m_edge_count;
}

void entity_graph::remove_all_edges(index_type node_index) {
    auto &node = m_nodes[node_index];
    auto adj_index = node.adjacency_index;
    node.adjacency_index = null_index;

    while (adj_index != null_index) {
        auto &adj = m_adjacencies[adj_index];
        auto edge_index = adj.edge_index;

        // Remove all edges.
        while (edge_index != null_index) {
            auto &edge = m_edges[edge_index];
            auto next = edge.next;
            edge.entity = entt::null;
            edge.next = m_edges_free_list;
            m_edges_free_list = edge_index;
            --m_edge_count;

            edge_index = next;
        }

        // Remove adjacency from neighbor.
        auto neighbor_node_index = adj.node_index;
        auto &neighbor = m_nodes[neighbor_node_index];
        auto neighbor_adj_index = neighbor.adjacency_index;

        while (neighbor_adj_index != null_index) {
            auto &neighbor_adj = m_adjacencies[neighbor_adj_index];
            if (neighbor_adj.node_index == node_index) {
                // All edges have been removed.
                neighbor_adj.edge_index = null_index;
                remove_adjacency(neighbor_node_index, neighbor_adj_index);
                break;
            }
            neighbor_adj_index = neighbor_adj.next;
        }

        adj_index = adj.next;

        // Remove adjacency.
        adj.node_index = null_index;
        adj.edge_index = null_index;
        adj.next = m_adjacencies_free_list;
        m_adjacencies_free_list = adj_index;
    }
}

entt::entity entity_graph::edge_entity(index_type edge_index) const {
    EDYN_ASSERT(edge_index < m_edges.size());
    EDYN_ASSERT(m_edges[edge_index].entity != entt::null);
    return m_edges[edge_index].entity;
}

bool entity_graph::has_adjacency(index_type node_index0, index_type node_index1) const {
    EDYN_ASSERT(node_index0 < m_nodes.size());
    EDYN_ASSERT(node_index1 < m_nodes.size());
    auto adj_index = m_nodes[node_index0].adjacency_index;

    while (adj_index != null_index) {
        if (m_adjacencies[adj_index].node_index == node_index1) {
            return true;
        }
        adj_index = m_adjacencies[adj_index].next;
    }

    return false;
}

std::array<entity_graph::index_type, 2> entity_graph::edge_node_indices(index_type edge_index) const {
    EDYN_ASSERT(edge_index < m_edges.size());
    auto &edge = m_edges[edge_index];
    EDYN_ASSERT(edge.entity != entt::null);
    return {edge.node_index0, edge.node_index1};
}

entity_pair entity_graph::edge_node_entities(index_type edge_index) const {
    EDYN_ASSERT(edge_index < m_edges.size());
    auto &edge = m_edges[edge_index];
    EDYN_ASSERT(edge.entity != entt::null);
    return {m_nodes[edge.node_index0].entity, m_nodes[edge.node_index1].entity};
}

bool entity_graph::is_connecting_node(index_type node_index) const {
    EDYN_ASSERT(node_index < m_nodes.size());
    return !m_nodes[node_index].non_connecting;
}

void entity_graph::set_connecting_node(index_type node_index, bool connecting) {
    EDYN_ASSERT(node_index < m_nodes.size());
    m_nodes[node_index].non_connecting = !connecting;
}

void entity_graph::insert_adjacency(index_type node_index0, index_type node_index1, index_type edge_index) {
    m_edges[edge_index].adj_index0 = insert_adjacency_one_way(node_index0, node_index1, edge_index);

    if (node_index0 != node_index1) {
        m_edges[edge_index].adj_index1 = insert_adjacency_one_way(node_index1, node_index0, edge_index);
    }
}

entity_graph::index_type entity_graph::insert_adjacency_one_way(index_type node_index0, index_type node_index1, index_type edge_index) {
    // Should not have duplicate adjacencies.
    EDYN_ASSERT(!has_adjacency(node_index0, node_index1));

    auto adj_index = create_adjacency(node_index1, edge_index);
    m_adjacencies[adj_index].next = m_nodes[node_index0].adjacency_index;
    m_nodes[node_index0].adjacency_index = adj_index;
    return adj_index;
}

entity_graph::index_type entity_graph::create_adjacency(index_type destination_node_index, index_type edge_index) {
    if (m_adjacencies_free_list == null_index) {
        m_adjacencies_free_list = m_adjacencies.size();
        m_adjacencies.resize(m_adjacencies.size() + allocation_size);

        for (auto i = m_adjacencies_free_list; i < m_adjacencies.size(); ++i) {
            m_adjacencies[i].next = i + 1;
            m_adjacencies[i].node_index = null_index;
            m_adjacencies[i].edge_index = null_index;
        }

        m_adjacencies.back().next = null_index;
    }

    auto index = m_adjacencies_free_list;
    auto &adj = m_adjacencies[index];
    m_adjacencies_free_list = adj.next;
    adj.node_index = destination_node_index;
    adj.edge_index = edge_index;
    adj.next = null_index;

    return index;
}

void entity_graph::remove_adjacency_edge(index_type source_node_index, index_type adj_index, index_type edge_index) {
    auto &adj = m_adjacencies[adj_index];
    const auto &edge = m_edges[edge_index];

    if (adj.edge_index == edge_index) {
        // Edge being removed is the first in the linked list.
        adj.edge_index = edge.next;
    }

    // Remove adjacency if it doesn't have any edges.
    if (adj.edge_index == null_index) {
        remove_adjacency(source_node_index, adj_index);
    }
}

void entity_graph::remove_adjacency(index_type source_node_index, index_type adj_index) {
    EDYN_ASSERT(adj_index < m_adjacencies.size());
    auto &adj = m_adjacencies[adj_index];

    // All edges must have been removed first.
    EDYN_ASSERT(adj.edge_index == null_index);

    // Remove from node's adjacency list.
    auto &node = m_nodes[source_node_index];
    auto idx = node.adjacency_index;

    if (idx == adj_index) {
        // Adjacency being removed is the first in the adjacency linked list.
        node.adjacency_index = adj.next;
    } else {
        // Find adjacency before the one being removed.
        while (m_adjacencies[idx].next != adj_index) {
            EDYN_ASSERT(m_adjacencies[idx].next != null_index);
            idx = m_adjacencies[idx].next;
        }
        EDYN_ASSERT(idx != null_index);
        m_adjacencies[idx].next = adj.next;
    }

    adj.node_index = null_index;
    adj.edge_index = null_index;
    adj.next = m_adjacencies_free_list;
    m_adjacencies_free_list = adj_index;
}

bool entity_graph::is_single_connected_component() const {
    EDYN_ASSERT(m_node_count > 0);

    std::vector<bool> visited;
    visited.assign(m_nodes.size(), false);
    std::vector<index_type> to_visit;

    for (index_type i = 0; i < m_nodes.size(); ++i) {
        if (m_nodes[i].entity != entt::null &&
            !m_nodes[i].non_connecting) {
            to_visit.push_back(i);
            break;
        }
    }

    while (!to_visit.empty()) {
        auto node_index = to_visit.back();
        to_visit.pop_back();

        visited[node_index] = true;
        auto &node = m_nodes[node_index];

        if (node.non_connecting) {
            continue;
        }

        auto adj_index = node.adjacency_index;

        while (adj_index != null_index) {
            auto neighbor_index = m_adjacencies[adj_index].node_index;

            if (!visited[neighbor_index]) {
                to_visit.push_back(neighbor_index);
                visited[neighbor_index] = true;
            }

            adj_index = m_adjacencies[adj_index].next;
        }
    }

    // Check if there's any one connecting node that has not been visited.
    for (size_t i = 0; i < visited.size(); ++i) {
        if (!visited[i] &&
            m_nodes[i].entity != entt::null &&
            !m_nodes[i].non_connecting) {
            return false;
        }
    }

    return true;
}

entity_graph::connected_components_t entity_graph::connected_components() const {
    auto components = entity_graph::connected_components_t{};

    std::vector<bool> visited;
    std::vector<bool> visited_edges;
    visited.assign(m_nodes.size(), false);
    visited_edges.assign(m_edges.size(), false);

    std::vector<index_type> to_visit;

    for (size_t node_index = 0; node_index < m_nodes.size(); ++node_index) {
        auto &node = m_nodes[node_index];
        if (node.entity != entt::null && !node.non_connecting) {
            to_visit.push_back(node_index);
            break;
        }
    }

    std::vector<index_type> non_connecting_indices;

    while (true) {
        auto &connected = components.emplace_back();

        // Visit nodes reachable from the initial node inserted into `to_visit`.
        while (!to_visit.empty()) {
            auto node_index = to_visit.back();
            to_visit.pop_back();

            visited[node_index] = true;

            const auto &node = m_nodes[node_index];
            connected.nodes.push_back(node.entity);

            if (node.non_connecting) {
                non_connecting_indices.push_back(node_index);
                continue;
            }

            auto adj_index = node.adjacency_index;

            while (adj_index != null_index) {
                auto &adj = m_adjacencies[adj_index];
                auto edge_index = adj.edge_index;

                while (edge_index != null_index) {
                    auto &edge = m_edges[edge_index];

                    if (!visited_edges[edge_index]) {
                        EDYN_ASSERT(edge.entity != entt::null);
                        connected.edges.push_back(edge.entity);
                        visited_edges[edge_index] = true;
                    }

                    edge_index = edge.next;
                }

                auto neighbor_index = adj.node_index;

                if (!visited[neighbor_index]) {
                    to_visit.push_back(neighbor_index);
                    // Mark as visited in advance to prevent inserting the same node index
                    // in the `to_visit` array more than once.
                    visited[neighbor_index] = true;
                }

                adj_index = adj.next;
            }
        }

        // Mark non-connecting nodes as unvisited so they'll be visited again
        // once while traversing the next connected component.
        for (auto node_index : non_connecting_indices) {
            visited[node_index] = false;
        }
        non_connecting_indices.clear();

        // Look for a connecting node that has not yet been visited.
        for (size_t node_index = 0; node_index < m_nodes.size(); ++node_index) {
            if (!visited[node_index] &&
                m_nodes[node_index].entity != entt::null &&
                !m_nodes[node_index].non_connecting) {
                to_visit.push_back(node_index);
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

double entity_graph::efficiency() const {
    if (m_nodes.empty()) {
        return 0;
    }

    return (double)m_node_count / (double)m_nodes.size();
}

void entity_graph::optimize() {
    // TODO: Remove all the rightmost free entries and sort the free lists.
    /*for (size_t i = m_nodes.size() - 1; i > 0; --i) {
        if (m_nodes[i].entity != entt::null) {
            m_nodes.resize(i + 1);

            break;
        }
    }*/
}

void entity_graph::optimize_if_needed() {
    if (efficiency() < 0.6) {
        optimize();
    }
}

void entity_graph::clear() {
    m_nodes_free_list = null_index;
    m_edges_free_list = null_index;
    m_adjacencies_free_list = null_index;
    m_node_count = 0;
    m_edge_count = 0;

    if (!m_nodes.empty()) {
        for (size_t i = 0; i < m_nodes.size(); ++i) {
            auto &node = m_nodes[i];
            node.next = i + 1;
            node.adjacency_index = null_index;
            node.entity = entt::null;
        }

        m_nodes.back().next = null_index;
        m_nodes_free_list = 0;
    }

    if (!m_edges.empty()) {
        for (size_t i = 0; i < m_edges.size(); ++i) {
            auto &edge = m_edges[i];
            edge.next = i + 1;
            edge.node_index0 = null_index;
            edge.node_index1 = null_index;
            edge.adj_index0 = null_index;
            edge.adj_index1 = null_index;
            edge.entity = entt::null;
        }

        m_edges.back().next = null_index;
        m_edges_free_list = 0;
    }

    if (!m_adjacencies.empty()) {
        for (size_t i = 0; i < m_adjacencies.size(); ++i) {
            m_adjacencies[i].next = i + 1;
            m_adjacencies[i].node_index = null_index;
            m_adjacencies[i].edge_index = null_index;
        }

        m_adjacencies.back().next = null_index;
        m_adjacencies_free_list = 0;
    }
}

}
