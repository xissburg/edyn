#include "../common/common.hpp"
#include "edyn/comp/tag.hpp"
#include "edyn/networking/networking.hpp"
#include "edyn/networking/packet/registry_snapshot.hpp"
#include "edyn/networking/util/input_state_history.hpp"
#include "edyn/networking/util/pool_snapshot.hpp"
#include "edyn/networking/util/pool_snapshot_data.hpp"

struct input {
    int value {};
};

template<typename Archive>
void serialize(Archive &archive, input &i) {
    archive(i.value);
}

TEST(networking_test, input_state_history) {
    auto registry = entt::registry{};

    auto ent0 = registry.create();
    registry.emplace<input>(ent0, 224);

    auto ent1 = registry.create();

    auto ent2 = registry.create();
    registry.emplace<input>(ent2, 997);

    auto entities = entt::sparse_set{};
    entities.emplace(ent0);
    entities.emplace(ent1);
    entities.emplace(ent2);

    // Entities must be networked to make this work.
    for (auto entity : entities) {
        registry.emplace<edyn::networked_tag>(entity);
    }

    auto history = std::make_shared<edyn::input_state_history<input>>();
    auto writer = edyn::input_state_history_writer_impl<input>(history);
    writer.emplace(registry, entities, 1);

    registry.get<input>(ent0).value = 11;
    writer.emplace(registry, entities, 2);

    registry.get<input>(ent0).value = 0;
    registry.get<input>(ent2).value = 1337;
    writer.emplace(registry, entities, 3);

    registry.get<input>(ent0).value = -98;
    registry.get<input>(ent2).value = 77;

    std::vector<entt::entity> input_entities;
    input_entities.push_back(ent0);
    input_entities.push_back(ent2);

    auto snapshot = edyn::packet::registry_snapshot{};
    snapshot.entities.insert(snapshot.entities.end(), entities.begin(), entities.end());
    auto pool_data = std::make_shared<edyn::pool_snapshot_data_impl<input>>();
    pool_data->insert(registry, input_entities.begin(), input_entities.end(), snapshot.entities);
    auto input_pool = edyn::pool_snapshot{};
    input_pool.ptr = pool_data;
    snapshot.pools.push_back(std::move(input_pool));

    writer.emplace(snapshot, entities, 4, 0);

    auto registry2 = entt::registry{};
    auto emap = edyn::entity_map{};

    for (auto entity : entities) {
        emap.insert(entity, registry2.create());
    }

    registry2.emplace<input>(emap.at(ent0));
    registry2.emplace<input>(emap.at(ent2));

    auto reader = edyn::input_state_history_reader_impl<input>(history, {});
    reader.import_latest(2, registry2, emap);

    ASSERT_EQ(registry2.get<input>(emap.at(ent0)).value, 11);
    ASSERT_EQ(registry2.get<input>(emap.at(ent2)).value, 997);

    reader.import_each(2, 1, registry2, emap);

    ASSERT_EQ(registry2.get<input>(emap.at(ent0)).value, 0);
    ASSERT_EQ(registry2.get<input>(emap.at(ent2)).value, 1337);

    writer.erase_until(2);
    reader.import_latest(2, registry2, emap);

    ASSERT_EQ(registry2.get<input>(emap.at(ent0)).value, 0);
    ASSERT_EQ(registry2.get<input>(emap.at(ent2)).value, 1337);

    reader.import_each(2, 2, registry2, emap);

    ASSERT_EQ(registry2.get<input>(emap.at(ent0)).value, -98);
    ASSERT_EQ(registry2.get<input>(emap.at(ent2)).value, 77);
}
