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
    auto entities = std::vector<entt::entity>();
    entities.push_back(ent0);
    entities.push_back(ent1);
    writer.component<int, edyn::vector3>(entities.begin(), entities.end());
    edyn::serialize(output, writer);

    reg0.replace<int>(ent0, 335);
    reg0.replace<edyn::vector3>(ent1, edyn::vector3_zero);
    
    auto input = edyn::memory_input_archive(buffer);
    auto reader = edyn::registry_snapshot_reader<int, edyn::vector3>(reg0);
    edyn::serialize(input, reader);

    ASSERT_EQ(reg0.get<int>(ent0), 665);
    ASSERT_EQ(reg0.get<edyn::vector3>(ent0), (edyn::vector3{3, 2, -1}));
    ASSERT_EQ(reg0.get<edyn::vector3>(ent1), (edyn::vector3{1, 2, 3}));
}

TEST(registry_serialization_test, test_registry_export_import) {
    entt::registry reg0;
    auto ent0 = reg0.create();
    reg0.assign<int>(ent0, 665);
    reg0.assign<edyn::vector3>(ent0, 3, 2, -1);
    auto ent1 = reg0.create();
    reg0.assign<edyn::vector3>(ent1, 1, 2, 3);

    auto buffer = edyn::memory_output_archive::buffer_type{};
    auto output = edyn::memory_output_archive(buffer);
    auto writer = edyn::registry_snapshot_writer<int, edyn::vector3>(reg0);
    auto entities = std::vector<entt::entity>();
    entities.push_back(ent0);
    entities.push_back(ent1);
    writer.component<int, edyn::vector3>(entities.begin(), entities.end());
    edyn::serialize(output, writer);

    entt::registry reg1;
    edyn::entity_map map;
    auto input = edyn::memory_input_archive(buffer);
    auto importer = edyn::registry_snapshot_importer<int, edyn::vector3>(reg1, map);
    edyn::serialize(input, importer);

    ASSERT_EQ(reg1.get<int>(map[ent0]), 665);
    ASSERT_EQ(reg1.get<edyn::vector3>(map[ent0]), (edyn::vector3{3, 2, -1}));
    ASSERT_EQ(reg1.get<edyn::vector3>(map[ent1]), (edyn::vector3{1, 2, 3}));

    // Replace some entities in `reg1`, export it and load it into `reg0`.
    reg1.replace<edyn::vector3>(map[ent1], 4, 5, 6);
    reg1.replace<int>(map[ent0], 1023);

    edyn::entity_map unmap;
    for (auto &pair : map) {
        unmap[pair.second] = pair.first;
    }
    auto buffer1 = edyn::memory_output_archive::buffer_type{};
    auto output1 = edyn::memory_output_archive(buffer1);
    auto exporter = edyn::registry_snapshot_exporter<int, edyn::vector3>(reg1, unmap);
    auto entities1 = std::vector<entt::entity>();
    entities1.push_back(map[ent0]);
    entities1.push_back(map[ent1]);
    exporter.component<int, edyn::vector3>(entities1.begin(), entities1.end());
    edyn::serialize(output1, exporter); 

    auto input0 = edyn::memory_input_archive(buffer1);
    auto reader = edyn::registry_snapshot_reader<int, edyn::vector3>(reg0);
    edyn::serialize(input0, reader);

    ASSERT_EQ(reg0.get<int>(ent0), 1023);
    ASSERT_EQ(reg0.get<edyn::vector3>(ent0), (edyn::vector3{3, 2, -1}));
    ASSERT_EQ(reg0.get<edyn::vector3>(ent1), (edyn::vector3{4, 5, 6}));
}