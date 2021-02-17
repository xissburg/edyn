#ifndef EDYN_PARALLEL_GRAPH_HPP
#define EDYN_PARALLEL_GRAPH_HPP

#include "edyn/config/config.h"
#include <vector>
#include <cstdint>
#include <entt/fwd.hpp>
#include <entt/entity/entity.hpp>

namespace edyn {

class graph final {
public:
    using index_type = size_t;
    constexpr static index_type null_index = SIZE_MAX;

private:
    struct node {
        entt::entity entity;
        index_type adjacency_index;
        index_type next;
    };

    struct edge {
        entt::entity entity;
        index_type node_index0, node_index1;
        index_type next;
    };

    struct adjacency {
        index_type node_index;
        index_type next;
    };

public:
    index_type insert_node(entt::entity entity) {
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

    void remove_node(index_type index) {
        EDYN_ASSERT(index < m_nodes.size());
        EDYN_ASSERT(m_nodes[index].entity != entt::null);

        m_nodes[index].entity = entt::null;
        m_nodes[index].next = m_nodes_free_list;
        m_nodes_free_list = index;
        --m_node_count;
    }

    index_type insert_edge(entt::entity entity, index_type node_index0, index_type node_index1) {
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

        auto index = m_edges_free_list;
        m_edges_free_list = m_edges[index].next;
        m_edges[index].entity = entity;
        m_edges[index].node_index0 = node_index0;
        m_edges[index].node_index1 = node_index1;

        insert_adjacency(node_index0, node_index1);

        ++m_edge_count;

        return index;
    }

    void remove_edge(index_type index) {

    }

    void insert_adjacency(index_type node_index0, index_type node_index1) {
        insert_adjacency_one_way(node_index0, node_index1);
        insert_adjacency_one_way(node_index1, node_index0);
    }

    void insert_adjacency_one_way(index_type node_index0, index_type node_index1) {
        if (m_nodes[node_index0].adjacency_index == null_index) {
            m_nodes[node_index0].adjacency_index = create_adjacency(node_index1);
        } else {
            auto adj = m_nodes[node_index0].adjacency_index;

            while (m_adjacencies[adj].next != null_index) {
                adj = m_adjacencies[adj].next;
            }

            m_adjacencies[adj].next = create_adjacency(node_index1);
        }
    }

    index_type create_adjacency(index_type node_index) {
        if (m_adjacencies_free_list == null_index) {
            m_adjacencies_free_list = m_adjacencies.size();
            m_adjacencies.resize(m_adjacencies.size() + 16);

            for (auto i = m_adjacencies_free_list; i < m_adjacencies.size() - 1; ++i) {
                m_adjacencies[i].next = i + 1;
                m_adjacencies[i].node_index = null_index;
            }

            m_adjacencies.back().next = null_index;
        }

        auto index = m_adjacencies_free_list;
        m_adjacencies_free_list = m_adjacencies[index].next;
        m_adjacencies[index].node_index = node_index;

        return index;
    }

    bool is_single_connected_component() const {
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

            auto adj = m_nodes[node_index].adjacency_index;

            while (adj != null_index) {
                auto neighbor_index = m_adjacencies[adj].node_index;
                to_visit.push_back(neighbor_index);
                adj = m_adjacencies[adj].next;
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

    double efficiency() const {
        if (m_nodes.empty()) {
            return 0;
        }

        return (double)m_node_count / (double)m_nodes.size();
    }

    void optimize() {
        // Remove all the rightmost free entries and sort the free lists.
    }

    void optimize_if_needed() {
        if (efficiency() < 0.6) {
            optimize();
        }
    }

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

}

#endif // EDYN_PARALLEL_GRAPH_HPP