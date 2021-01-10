#include "../common/common.hpp"
#include <tuple>
#include <memory>

struct custom_component {
    edyn::scalar value;
    entt::entity entity;
};

// `custom_component` needs a custom merge function to map its entity into the
// context of the other registry where it's being imported into.
namespace edyn {
    template<merge_type MergeType>
    void merge(const custom_component *old_comp, custom_component &new_comp, merge_context &ctx) {
        new_comp.entity = ctx.map->remloc(new_comp.entity);
    }
}

TEST(registry_delta_test, test_registry_delta_export_import) {
    edyn::register_external_components<custom_component>();
    edyn::init();

    entt::registry reg0;
    auto child0 = reg0.create();
    auto child1 = reg0.create();
    auto ent0 = reg0.create();
    reg0.emplace<edyn::island_node>(ent0, edyn::entity_set{child0, child1});
    auto ent1 = reg0.create();
    reg0.emplace<edyn::contact_point>(ent1, std::array<entt::entity, 2>{child0, child1});
    reg0.get<edyn::contact_point>(ent1).distance = 6.28;
    auto ent2 = reg0.create();
    reg0.emplace<custom_component>(ent2, 3.14, child0);

    auto map0 = edyn::entity_map{};
    auto builder = edyn::make_registry_delta_builder(map0);
    builder->created(ent0);
    builder->created(ent0, reg0.get<edyn::island_node>(ent0));
    builder->created(ent1);
    builder->created(ent1, reg0.get<edyn::contact_point>(ent1));
    builder->created(child0);
    builder->created(child1);
    builder->created(ent2);
    builder->created_all(ent2, reg0);

    entt::registry reg1;
    auto map1 = edyn::entity_map{};
    builder->get_delta().import(reg1, map1);

    auto builder1 = edyn::make_registry_delta_builder(map1);

    // `map1` contains the entity mapping between reg0 and reg1 (corresponding
    // entities are created on import and mappings are added to `map1`).
    // It is necessary to insert these mappings in `builder1` so when the delta
    // is exported and then imported into `reg0`, it can map the entities back.
    for (auto remote_entity : builder->get_delta().created_entities()) {
        auto local_entity = map1.remloc(remote_entity);
        builder1->insert_entity_mapping(local_entity);
    }

    ASSERT_TRUE(reg1.get<edyn::island_node>(map1.remloc(ent0)).entities.contains(map1.remloc(child0)));
    ASSERT_TRUE(reg1.get<edyn::island_node>(map1.remloc(ent0)).entities.contains(map1.remloc(child1)));
    ASSERT_EQ(map1.locrem(reg1.get<edyn::contact_point>(map1.remloc(ent1)).body[0]), child0);
    ASSERT_SCALAR_EQ(reg1.get<edyn::contact_point>(map1.remloc(ent1)).distance, 6.28);
    ASSERT_SCALAR_EQ(reg1.get<custom_component>(map1.remloc(ent2)).value, 3.14);
    ASSERT_EQ(reg1.get<custom_component>(map1.remloc(ent2)).entity, map1.remloc(child0));

    // Replace some entities in `reg1`, export it and load it into `reg0`.
    auto &comp0 = reg1.get<edyn::island_node>(map1.remloc(ent0));
    comp0.entities.erase(map1.remloc(child0));
    comp0.entities.insert(map1.remloc(ent1));
    builder1->updated(map1.remloc(ent0), comp0);

    auto &custom = reg1.get<custom_component>(map1.remloc(ent2));
    custom.entity = map1.remloc(ent2);
    builder1->updated_all(map1.remloc(ent2), reg1);
    
    builder1->get_delta().import(reg0, map0);

    ASSERT_TRUE(reg0.get<edyn::island_node>(ent0).entities.contains(ent1));
    ASSERT_TRUE(reg0.get<edyn::island_node>(ent0).entities.contains(child1));
    ASSERT_EQ(reg0.get<edyn::contact_point>(ent1).body[0], child0);
    ASSERT_SCALAR_EQ(reg0.get<edyn::contact_point>(ent1).distance, 6.28);
    ASSERT_EQ(reg0.get<custom_component>(ent2).entity, ent2);
}