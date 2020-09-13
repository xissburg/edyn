#include "../common/common.hpp"

TEST(registry_serialization_test, test_registry_writer_reader) {
    entt::registry reg0;
    auto ent0 = reg0.create();
    reg0.assign<int>(ent0, 665);
    reg0.assign<edyn::vector3>(ent0, 3, 2, -1);
    auto ent1 = reg0.create();
    reg0.assign<edyn::vector3>(ent1, 1, 2, 3);

    auto buffer = edyn::memory_output_archive::buffer_type{};
    auto output = edyn::memory_output_archive(buffer);
    auto writer = edyn::registry_snapshot_writer<int, edyn::vector3>(reg0);
    writer.updated<int, edyn::vector3>(ent0);
    writer.updated<edyn::vector3>(ent1);
    writer.serialize(output);

    reg0.replace<int>(ent0, 335);
    reg0.replace<edyn::vector3>(ent1, edyn::vector3_zero);
    
    auto input = edyn::memory_input_archive(buffer);
    auto reader = edyn::registry_snapshot_reader<int, edyn::vector3>(reg0);
    reader.serialize(input);

    ASSERT_EQ(reg0.get<int>(ent0), 665);
    ASSERT_EQ(reg0.get<edyn::vector3>(ent0), (edyn::vector3{3, 2, -1}));
    ASSERT_EQ(reg0.get<edyn::vector3>(ent1), (edyn::vector3{1, 2, 3}));
}

struct component_with_child {
    entt::entity child;
    int i;
};

template<typename Archive>
void serialize(Archive &archive, component_with_child &c) {
    archive(c.child);
    archive(c.i);
}

struct component_with_children {
    std::array<entt::entity, 3> children;
};

template<typename Archive>
void serialize(Archive &archive, component_with_children &c) {
    archive(c.children);
}

TEST(registry_serialization_test, test_registry_export_import) {
    entt::registry reg0;
    auto child0 = reg0.create();
    auto child1 = reg0.create();
    auto ent0 = reg0.create();
    reg0.assign<component_with_child>(ent0, child0, 665);
    auto ent1 = reg0.create();
    reg0.assign<component_with_children>(ent1, child0, child1, ent0);

    auto buffer = edyn::memory_output_archive::buffer_type{};
    auto output = edyn::memory_output_archive(buffer);
    auto writer = edyn::registry_snapshot_writer<component_with_child, component_with_children>(reg0);
    writer.updated<component_with_child>(ent0);
    writer.updated<component_with_children>(ent1);
    writer.updated(child0);
    writer.updated(child1);
    writer.serialize(output);

    entt::registry reg1;
    edyn::entity_map map;
    auto input = edyn::memory_input_archive(buffer);
    auto importer = edyn::registry_snapshot_importer<component_with_child, component_with_children>(reg1, map);
    importer.serialize(input, &component_with_child::child, &component_with_children::children);

    ASSERT_EQ(reg1.get<component_with_child>(map.remloc(ent0)).i, 665);
    ASSERT_EQ(map.locrem(reg1.get<component_with_child>(map.remloc(ent0)).child), child0);
    ASSERT_EQ(map.locrem(reg1.get<component_with_children>(map.remloc(ent1)).children[0]), child0);
    ASSERT_EQ(map.locrem(reg1.get<component_with_children>(map.remloc(ent1)).children[1]), child1);
    ASSERT_EQ(map.locrem(reg1.get<component_with_children>(map.remloc(ent1)).children[2]), ent0);

    // Replace some entities in `reg1`, export it and load it into `reg0`.
    auto &comp0 = reg1.get<component_with_child>(map.remloc(ent0));
    comp0.i = 1025;
    comp0.child = map.remloc(ent1);

    auto buffer1 = edyn::memory_output_archive::buffer_type{};
    auto output1 = edyn::memory_output_archive(buffer1);
    auto exporter = edyn::registry_snapshot_exporter<component_with_child, component_with_children>(reg1, map);
    exporter.updated<component_with_child>(ent0);
    exporter.updated<component_with_children>(ent1);
    exporter.serialize(output1, &component_with_child::child, &component_with_children::children);

    auto input0 = edyn::memory_input_archive(buffer1);
    auto reader = edyn::registry_snapshot_reader<component_with_child, component_with_children>(reg0);
    reader.serialize(input0);

    ASSERT_EQ(reg0.get<component_with_child>(ent0).i, 1025);
    ASSERT_EQ(reg0.get<component_with_child>(ent0).child, ent1);
    ASSERT_EQ(reg0.get<component_with_children>(ent1).children[0], child0);
    ASSERT_EQ(reg0.get<component_with_children>(ent1).children[1], child1);
    ASSERT_EQ(reg0.get<component_with_children>(ent1).children[2], ent0);
}