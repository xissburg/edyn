#include "../common/common.hpp"
#include <tuple>
#include <memory>
#include <entt/meta/factory.hpp>
#include <entt/core/hashed_string.hpp>

struct custom_component {
    edyn::scalar value;
    entt::entity entity;
};

struct parent_component {
    std::array<entt::entity, 2> entity;
};

template<typename T, size_t N>
bool array_contains(std::array<T, N> &arr, const T &val) {
    return std::find(arr.begin(), arr.end(), val) != arr.end();
}

TEST(island_delta_test, test_island_delta_export_import) {
    entt::registry reg0;
    edyn::init();
    edyn::attach(reg0);
    edyn::register_external_components<custom_component, parent_component>(reg0);

    using namespace entt::literals;
    entt::meta<custom_component>().type()
        .data<&custom_component::entity, entt::as_ref_t>("entity"_hs);
    entt::meta<parent_component>().type()
        .data<&parent_component::entity, entt::as_ref_t>("entity"_hs);

    auto child0 = reg0.create();
    auto child1 = reg0.create();
    auto ent0 = reg0.create();
    reg0.emplace<parent_component>(ent0, std::array<entt::entity, 2>{child0, child1});
    auto ent1 = reg0.create();
    reg0.emplace<edyn::distance_constraint>(ent1, std::array<entt::entity, 2>{child0, child1});
    reg0.get<edyn::distance_constraint>(ent1).distance = 6.28f;
    auto ent2 = reg0.create();
    reg0.emplace<custom_component>(ent2, 3.14f, child0);

    auto map0 = edyn::entity_map{};
    auto builder = edyn::make_island_delta_builder(reg0);
    builder->created(ent0);
    builder->created(ent0, reg0.get<parent_component>(ent0));
    builder->created(ent1);
    builder->created(ent1, reg0.get<edyn::distance_constraint>(ent1));
    builder->created(child0);
    builder->created(child1);
    builder->created(ent2);
    builder->created_all(ent2, reg0);

    entt::registry reg1;
    edyn::attach(reg1);
    edyn::register_external_components<custom_component, parent_component>(reg1);

    auto map1 = edyn::entity_map{};

    auto delta = builder->finish();
    delta.import(reg1, map1);

    auto builder1 = edyn::make_island_delta_builder(reg1);

    // `map1` contains the entity mapping between reg0 and reg1 (corresponding
    // entities are created on import and mappings are added to `map1`).
    // It is necessary to insert these mappings in `builder1` so when the delta
    // is exported and then imported into `reg0`, it can map the entities back.
    for (auto remote_entity : delta.created_entities()) {
        auto local_entity = map1.at(remote_entity);
        builder1->insert_entity_mapping(remote_entity, local_entity);
    }

    ASSERT_TRUE(array_contains(reg1.get<parent_component>(map1.at(ent0)).entity, map1.at(child0)));
    ASSERT_TRUE(array_contains(reg1.get<parent_component>(map1.at(ent0)).entity, map1.at(child1)));
    ASSERT_EQ(map1.at(child0), reg1.get<edyn::distance_constraint>(map1.at(ent1)).body[0]);
    ASSERT_SCALAR_EQ(reg1.get<edyn::distance_constraint>(map1.at(ent1)).distance, 6.28f);
    ASSERT_SCALAR_EQ(reg1.get<custom_component>(map1.at(ent2)).value, 3.14f);
    ASSERT_EQ(reg1.get<custom_component>(map1.at(ent2)).entity, map1.at(child0));

    // Replace some entities in `reg1`, export it and load it into `reg0`.
    auto &comp0 = reg1.get<parent_component>(map1.at(ent0));
    for (auto &entity : comp0.entity) {
        if (entity == map1.at(child0)) {
            entity = map1.at(ent1);
        }
    }
    builder1->updated(map1.at(ent0), comp0);

    auto &custom = reg1.get<custom_component>(map1.at(ent2));
    custom.entity = map1.at(ent2);
    builder1->updated_all(map1.at(ent2), reg1);

    builder1->finish().import(reg0, map0);

    ASSERT_TRUE(array_contains(reg0.get<parent_component>(ent0).entity, ent1));
    ASSERT_TRUE(array_contains(reg0.get<parent_component>(ent0).entity, child1));
    ASSERT_EQ(reg0.get<edyn::distance_constraint>(ent1).body[0], child0);
    ASSERT_SCALAR_EQ(reg0.get<edyn::distance_constraint>(ent1).distance, 6.28f);
    ASSERT_EQ(reg0.get<custom_component>(ent2).entity, ent2);

    edyn::detach(reg0);
    edyn::detach(reg1);
    edyn::deinit();
}
