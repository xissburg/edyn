#include "../common/common.hpp"

TEST(registry_snapshot_test, test_registry_updated_and_import) {
    edyn::init();

    entt::registry reg0;
    auto ent0 = reg0.create();
    reg0.emplace<edyn::orientation>(ent0, 665, 0, 0, 1);
    reg0.emplace<edyn::position>(ent0, 3, 2, -1);
    auto ent1 = reg0.create();
    reg0.emplace<edyn::position>(ent1, 1, 2, 3);

    auto e_map = edyn::entity_map{};
    e_map.insert(ent0, ent0);
    e_map.insert(ent1, ent1);

    auto builder = edyn::registry_snapshot_builder(e_map);
    builder.updated(ent0, reg0.get<edyn::orientation>(ent0), reg0.get<edyn::position>(ent0));
    builder.updated(ent1, reg0.get<edyn::position>(ent1));

    reg0.replace<edyn::orientation>(ent0, 335, 0, 0, 1);
    reg0.replace<edyn::position>(ent1, edyn::vector3_zero);

    builder.get_snapshot().import(reg0, e_map);

    ASSERT_EQ(reg0.get<edyn::orientation>(ent0).x, 665);
    ASSERT_EQ(reg0.get<edyn::position>(ent0), (edyn::position{{3, 2, -1}}));
    ASSERT_EQ(reg0.get<edyn::position>(ent1), (edyn::position{{1, 2, 3}}));
}

TEST(registry_snapshot_test, test_registry_export_import) {
    edyn::init();

    entt::registry reg0;
    auto child0 = reg0.create();
    auto child1 = reg0.create();
    auto ent0 = reg0.create();
    reg0.emplace<edyn::island_node>(ent0, std::vector<entt::entity>{child0, child1});
    auto ent1 = reg0.create();
    reg0.emplace<edyn::contact_point>(ent1, std::array<entt::entity, 2>{child0, child1});
    reg0.get<edyn::contact_point>(ent1).distance = 6.28;

    auto map0 = edyn::entity_map{};
    auto builder = edyn::registry_snapshot_builder(map0);
    builder.created(ent0);
    builder.updated(ent0, reg0.get<edyn::island_node>(ent0));
    builder.created(ent1);
    builder.updated(ent1, reg0.get<edyn::contact_point>(ent1));
    builder.created(child0);
    builder.created(child1);

    entt::registry reg1;
    auto map1 = edyn::entity_map{};
    builder.get_snapshot().import(reg1, map1);

    ASSERT_EQ(map1.locrem(reg1.get<edyn::island_node>(map1.remloc(ent0)).entities[0]), child0);
    ASSERT_EQ(map1.locrem(reg1.get<edyn::island_node>(map1.remloc(ent0)).entities[1]), child1);
    ASSERT_EQ(map1.locrem(reg1.get<edyn::contact_point>(map1.remloc(ent1)).body[0]), child0);
    ASSERT_SCALAR_EQ(reg1.get<edyn::contact_point>(map1.remloc(ent1)).distance, 6.28);

    // Replace some entities in `reg1`, export it and load it into `reg0`.
    auto &comp0 = reg1.get<edyn::island_node>(map1.remloc(ent0));
    comp0.entities[0] = map1.remloc(ent1);

    auto builder1 = edyn::registry_snapshot_builder(map1);
    builder1.updated(map1.remloc(ent0), reg1.get<edyn::island_node>(map1.remloc(ent0)));
    builder1.updated(map1.remloc(ent1), reg1.get<edyn::contact_point>(map1.remloc(ent1)));
    
    builder1.get_snapshot().import(reg0, map0);

    ASSERT_EQ(reg0.get<edyn::island_node>(ent0).entities[0], ent1);
    ASSERT_EQ(reg0.get<edyn::island_node>(ent0).entities[1], child1);
    ASSERT_EQ(reg0.get<edyn::contact_point>(ent1).body[0], child0);
    ASSERT_SCALAR_EQ(reg0.get<edyn::contact_point>(ent1).distance, 6.28);
}