#include "../common/common.hpp"

TEST(registry_delta_test, test_registry_delta_export_import) {
    edyn::init();

    entt::registry reg0;
    auto child0 = reg0.create();
    auto child1 = reg0.create();
    auto ent0 = reg0.create();
    reg0.emplace<edyn::island_node>(ent0, edyn::entity_set{child0, child1});
    auto ent1 = reg0.create();
    reg0.emplace<edyn::contact_point>(ent1, std::array<entt::entity, 2>{child0, child1});
    reg0.get<edyn::contact_point>(ent1).distance = 6.28;

    auto map0 = edyn::entity_map{};
    auto builder = edyn::registry_delta_builder(map0);
    builder.created(ent0);
    builder.created(ent0, reg0.get<edyn::island_node>(ent0));
    builder.created(ent1);
    builder.created(ent1, reg0.get<edyn::contact_point>(ent1));
    builder.created(child0);
    builder.created(child1);

    entt::registry reg1;
    auto map1 = edyn::entity_map{};
    builder.get_delta().import(reg1, map1);

    auto builder1 = edyn::registry_delta_builder(map1);

    for (auto remote_entity : builder.get_delta().created()) {
        if (!map1.has_rem(remote_entity)) continue;
        auto local_entity = map1.remloc(remote_entity);
        builder1.insert_entity_mapping(local_entity);
    }

    ASSERT_GT(reg1.get<edyn::island_node>(map1.remloc(ent0)).entities.count(map1.remloc(child0)), 0);
    ASSERT_GT(reg1.get<edyn::island_node>(map1.remloc(ent0)).entities.count(map1.remloc(child1)), 0);
    ASSERT_EQ(map1.locrem(reg1.get<edyn::contact_point>(map1.remloc(ent1)).body[0]), child0);
    ASSERT_SCALAR_EQ(reg1.get<edyn::contact_point>(map1.remloc(ent1)).distance, 6.28);

    // Replace some entities in `reg1`, export it and load it into `reg0`.
    auto &comp0 = reg1.get<edyn::island_node>(map1.remloc(ent0));
    comp0.entities.erase(map1.remloc(child0));
    comp0.entities.insert(map1.remloc(ent1));

    builder1.updated(map1.remloc(ent0), reg1.get<edyn::island_node>(map1.remloc(ent0)));
    builder1.updated(map1.remloc(ent1), reg1.get<edyn::contact_point>(map1.remloc(ent1)));
    
    builder1.get_delta().import(reg0, map0);

    ASSERT_GT(reg0.get<edyn::island_node>(ent0).entities.count(ent1), 0);
    ASSERT_GT(reg0.get<edyn::island_node>(ent0).entities.count(child1), 0);
    ASSERT_EQ(reg0.get<edyn::contact_point>(ent1).body[0], child0);
    ASSERT_SCALAR_EQ(reg0.get<edyn::contact_point>(ent1).distance, 6.28);
}