#include "edyn/parallel/entity_graph.hpp"
#include "edyn/config/config.h"

namespace edyn {

entity_graph::index_type entity_graph::insert_node(entt::entity entity) {
    if (m_nodes_free_list == null_index) {
        m_nodes_free_list = m_nodes.size();
        m_nodes.resize(m_nodes.size() + 16);

        for (auto i = m_nodes_free_list; i < m_nodes.size() - 1; ++i) {
            m_nodes[i].next = i + 1;
            m_nodes[i].entity = entt::null;
        }

        m_nodes.back().next = null_index;
    }

    auto index = m_nodes_free_list;
    m_nodes_free_list = m_nodes[index].next;
    m_nodes[index].entity = entity;
    ++m_node_count;

    return index;
}

void entity_graph::remove_node(index_type node_index) {
    EDYN_ASSERT(node_index < m_nodes.size());
    EDYN_ASSERT(m_nodes[node_index].entity != entt::null);

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
    EDYN_ASSERT(node_index0 < m_nodes.size() && node_index1 < m_nodes.size());
    EDYN_ASSERT(m_nodes[node_index0].entity != entt::null);
    EDYN_ASSERT(m_nodes[node_index1].entity != entt::null);

    if (m_edges_free_list == null_index) {
        m_edges_free_list = m_edges.size();
        m_edges.resize(m_edges.size() + 16);

        for (auto i = m_edges_free_list; i < m_edges.size() - 1; ++i) {
            m_edges[i].next = i + 1;
            m_edges[i].entity = entt::null;
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
    auto &node1 = m_nodes[edge.node_index1];

    remove_adjacency_edge(node0.adjacency_index, edge_index);
    remove_adjacency_edge(node1.adjacency_index, edge_index);

    edge.entity = entt::null;
    edge.next = m_edges_free_list;
    m_edges_free_list = edge_index;
    --m_edge_count;
}

entt::entity entity_graph::edge_entity(index_type edge_index) const {
    EDYN_ASSERT(edge_index < m_edges.size());
    EDYN_ASSERT(m_edges[edge_index].entity != entt::null);
    return m_edges[edge_index].entity;
}

bool entity_graph::has_edge(index_type node_index0, index_type node_index1) const {
    auto adj_index = m_nodes[node_index0].adjacency_index;

    while (adj_index != null_index) {
        if (m_adjacencies[adj_index].node_index == node_index1) {
            return true;
        }
        adj_index = m_adjacencies[adj_index].next;
    }

    return false;
}

void entity_graph::insert_adjacency(index_type node_index0, index_type node_index1, index_type edge_index) {
    m_edges[edge_index].adj_index0 = insert_adjacency_one_way(node_index0, node_index1, edge_index);
    m_edges[edge_index].adj_index1 = insert_adjacency_one_way(node_index1, node_index0, edge_index);
}

entity_graph::index_type entity_graph::insert_adjacency_one_way(index_type node_index0, index_type node_index1, index_type edge_index) {
    // Should not have duplicate adjacencies.
    EDYN_ASSERT(!has_edge(node_index0, node_index1));

    auto adj_index = create_adjacency(node_index1, edge_index);
    m_adjacencies[adj_index].next = m_nodes[node_index0].adjacency_index;
    m_nodes[node_index0].adjacency_index = adj_index;
    return adj_index;
}

entity_graph::index_type entity_graph::create_adjacency(index_type node_index, index_type edge_index) {
    if (m_adjacencies_free_list == null_index) {
        m_adjacencies_free_list = m_adjacencies.size();
        m_adjacencies.resize(m_adjacencies.size() + 16);

        for (auto i = m_adjacencies_free_list; i < m_adjacencies.size() - 1; ++i) {
            m_adjacencies[i].next = i + 1;
            m_adjacencies[i].node_index = null_index;
            m_adjacencies[i].edge_index = null_index;
        }

        m_adjacencies.back().next = null_index;
    }

    auto index = m_adjacencies_free_list;
    auto &adj = m_adjacencies[index];
    m_adjacencies_free_list = adj.next;
    adj.node_index = node_index;
    adj.edge_index = edge_index;
    adj.next = null_index;

    return index;
}

void entity_graph::remove_adjacency_edge(index_type adj_index, index_type edge_index) {
    auto &adj = m_adjacencies[adj_index];
    const auto &edge = m_edges[edge_index];

    if (adj.edge_index == edge_index) {
        // Edge being removed is the first in the linked list.
        adj.edge_index = edge.next;
    } else {
        // Find edge before the one being removed.
        auto prev_edge_index = adj.edge_index;

        while (m_edges[prev_edge_index].next != edge_index) {
            prev_edge_index = m_edges[prev_edge_index].next;
        }

        m_edges[prev_edge_index].next = edge.next;
    }

    if (adj.edge_index == null_index) {
        remove_adjacency(adj_index);
    }
}

void entity_graph::remove_adjacency(index_type adj_index) {
    EDYN_ASSERT(adj_index < m_adjacencies.size());
    auto &adj = m_adjacencies[adj_index];

    // All edges must have been removed first.
    EDYN_ASSERT(adj.edge_index == null_index);

    // Remove from node's adjacency list.
    auto &node = m_nodes[adj.node_index];
    auto idx = node.adjacency_index;

    if (idx == adj_index) {
        // Adjacency being removed is the first in the adjacency linked list.
        node.adjacency_index = adj.next;
    } else {
        // Find adjacency before the one being removed.
        while (m_adjacencies[idx].next != adj_index) {
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

    std::vector<bool> visited(m_nodes.size(), false);
    std::vector<index_type> to_visit;

    for (index_type i = 0; i < m_nodes.size(); ++i) {
        if (m_nodes[i].entity != entt::null) {
            to_visit.push_back(i);
            break;
        }
    }

    while (!to_visit.empty()) {
        auto node_index = to_visit.back();
        to_visit.pop_back();

        visited[node_index] = true;

        auto adj_index = m_nodes[node_index].adjacency_index;

        while (adj_index != null_index) {
            auto neighbor_index = m_adjacencies[adj_index].node_index;
            adj_index = m_adjacencies[adj_index].next;

            if (!visited[neighbor_index]) {
                to_visit.push_back(neighbor_index);
            }
        }
    }

    // Check if there's any one node that has not been visited.
    for (size_t i = 0; i < visited.size(); ++i) {
        if (!visited[i] && m_nodes[i].entity != entt::null) {
            return false;
        }
    }

    return true;
}

double entity_graph::efficiency() const {
    if (m_nodes.empty()) {
        return 0;
    }

    return (double)m_node_count / (double)m_nodes.size();
}

void entity_graph::optimize() {
    // Remove all the rightmost free entries and sort the free lists.

}

void entity_graph::optimize_if_needed() {
    if (efficiency() < 0.6) {
        optimize();
    }
}

}