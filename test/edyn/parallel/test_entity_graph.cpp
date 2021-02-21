#include "../common/common.hpp"
#include <edyn/parallel/entity_graph.hpp>

TEST(entity_graph_test, test_basics) {
    auto registry = entt::registry();
    auto graph = edyn::entity_graph();

    auto node_entity0 = registry.create();
    auto node_entity1 = registry.create();
    auto edge_entity01 = registry.create();

    auto node_index0 = graph.insert_node(node_entity0);
    auto node_index1 = graph.insert_node(node_entity1);
    auto edge_index01 = graph.insert_edge(edge_entity01, node_index0, node_index1);

    ASSERT_TRUE(graph.is_single_connected_component());
    
    graph.visit_neighbors(node_index0, [&] (entt::entity neighbor) {
        ASSERT_TRUE(neighbor == node_entity1);
    });

    std::vector<edyn::entity_graph::index_type> node_indices;
    node_indices.push_back(node_index0);
    node_indices.push_back(node_index1);

    auto connected_components = graph.connected_components(
        node_indices.begin(), node_indices.end(), 
        [] (auto, auto) { return true; });

    ASSERT_EQ(connected_components.size(), 1);

    ASSERT_NE(std::find(
        connected_components.front().nodes.begin(),
        connected_components.front().nodes.end(), node_entity0),
        connected_components.front().nodes.end());

    ASSERT_NE(std::find(
        connected_components.front().nodes.begin(),
        connected_components.front().nodes.end(), node_entity1),
        connected_components.front().nodes.end());

    ASSERT_EQ(connected_components.front().edges[0], edge_entity01);
}