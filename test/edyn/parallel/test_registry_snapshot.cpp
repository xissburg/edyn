#include "../common/common.hpp"

TEST(registry_snapshot_test, test_registry_updated_and_load) {
    entt::registry reg0;
    auto ent0 = reg0.create();
    reg0.assign<int>(ent0, 665);
    reg0.assign<edyn::vector3>(ent0, 3, 2, -1);
    auto ent1 = reg0.create();
    reg0.assign<edyn::vector3>(ent1, 1, 2, 3);

    auto snapshot = edyn::registry_snapshot<int, edyn::vector3>();
    snapshot.updated(ent0, reg0.get<int>(ent0), reg0.get<edyn::vector3>(ent0));
    snapshot.updated(ent1, reg0.get<edyn::vector3>(ent1));

    reg0.replace<int>(ent0, 335);
    reg0.replace<edyn::vector3>(ent1, edyn::vector3_zero);
    
    snapshot.load(reg0);

    ASSERT_EQ(reg0.get<int>(ent0), 665);
    ASSERT_EQ(reg0.get<edyn::vector3>(ent0), (edyn::vector3{3, 2, -1}));
    ASSERT_EQ(reg0.get<edyn::vector3>(ent1), (edyn::vector3{1, 2, 3}));
}

struct component_with_child {
    entt::entity child;
    int i;
};

struct component_with_children {
    std::array<entt::entity, 3> children;
};

TEST(registry_snapshot_test, test_registry_export_import) {
    entt::registry reg0;
    auto child0 = reg0.create();
    auto child1 = reg0.create();
    auto ent0 = reg0.create();
    reg0.assign<component_with_child>(ent0, child0, 665);
    auto ent1 = reg0.create();
    reg0.assign<component_with_children>(ent1, child0, child1, ent0);

    using components_tuple = std::tuple<component_with_child, component_with_children>;
    auto snapshot = edyn::registry_snapshot(components_tuple{});
    snapshot.updated(ent0, reg0.get<component_with_child>(ent0));
    snapshot.updated(ent1, reg0.get<component_with_children>(ent1));
    snapshot.updated(child0);
    snapshot.updated(child1);

    entt::registry reg1;
    edyn::entity_map map;
    snapshot.import(reg1, map, &component_with_child::child, &component_with_children::children);

    ASSERT_EQ(reg1.get<component_with_child>(map.remloc(ent0)).i, 665);
    ASSERT_EQ(map.locrem(reg1.get<component_with_child>(map.remloc(ent0)).child), child0);
    ASSERT_EQ(map.locrem(reg1.get<component_with_children>(map.remloc(ent1)).children[0]), child0);
    ASSERT_EQ(map.locrem(reg1.get<component_with_children>(map.remloc(ent1)).children[1]), child1);
    ASSERT_EQ(map.locrem(reg1.get<component_with_children>(map.remloc(ent1)).children[2]), ent0);

    // Replace some entities in `reg1`, export it and load it into `reg0`.
    auto &comp0 = reg1.get<component_with_child>(map.remloc(ent0));
    comp0.i = 1025;
    comp0.child = map.remloc(ent1);

    auto snapshot1 = edyn::registry_snapshot(components_tuple{});
    snapshot1.updated(map.remloc(ent0), reg1.get<component_with_child>(map.remloc(ent0)));
    snapshot1.updated(map.remloc(ent1), reg1.get<component_with_children>(map.remloc(ent1)));
    auto snapshot0 = snapshot1.map_entities(map, &component_with_child::child, &component_with_children::children);

    snapshot0.load(reg0);
    
    ASSERT_EQ(reg0.get<component_with_child>(ent0).i, 1025);
    ASSERT_EQ(reg0.get<component_with_child>(ent0).child, ent1);
    ASSERT_EQ(reg0.get<component_with_children>(ent1).children[0], child0);
    ASSERT_EQ(reg0.get<component_with_children>(ent1).children[1], child1);
    ASSERT_EQ(reg0.get<component_with_children>(ent1).children[2], ent0);
}