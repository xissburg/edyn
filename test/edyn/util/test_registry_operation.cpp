#include "../common/common.hpp"
#include "edyn/util/registry_operation.hpp"
#include <entt/meta/factory.hpp>
#include <entt/core/hashed_string.hpp>

TEST(test_registry_operation, test_create_destroy) {
    auto reg0 = entt::registry{};
    auto reg1 = entt::registry{};

    auto ent0 = reg0.create();

    edyn::registry_operation opc;
    opc.operation = edyn::registry_op_type::create;
    opc.entities.push_back(ent0);

    auto emap = edyn::entity_map{};
    // Should create a corresponding entity in reg1 and add it to the emap.
    opc.execute(reg1, emap, false);

    ASSERT_TRUE(emap.count(ent0));
    auto ent1 = emap.at(ent0);
    ASSERT_TRUE(reg1.valid(ent1));

    edyn::registry_operation opd;
    opd.operation = edyn::registry_op_type::destroy;
    opd.entities.push_back(ent0);
    // Should destroy entity in reg1 and remove it from emap.
    opd.execute(reg1, emap, false);

    ASSERT_FALSE(emap.count(ent0));
    ASSERT_FALSE(reg1.valid(ent1));
}

struct comp_with_entity {
    entt::entity entity;
};

TEST(test_registry_operation, test_components) {
    using namespace entt::literals;
    entt::meta<comp_with_entity>().type()
        .data<&comp_with_entity::entity, entt::as_ref_t>("entity"_hs);

    auto reg0 = entt::registry{};
    auto reg1 = entt::registry{};

    auto ent00 = reg0.create();
    auto ent01 = reg0.create();
    reg0.emplace<comp_with_entity>(ent00, ent01);

    auto builder = edyn::registry_operations_builder_impl{};
    builder.create(ent00);
    builder.create(ent01);
    builder.emplace<comp_with_entity>(reg0, ent00);
    auto ops = builder.finish();

    auto emap = edyn::entity_map{};
    ops.execute(reg1, emap, false);

    ASSERT_TRUE(emap.count(ent00));
    auto ent10 = emap.at(ent00);
    ASSERT_TRUE(reg1.valid(ent10));

    ASSERT_TRUE(emap.count(ent01));
    auto ent11 = emap.at(ent01);
    ASSERT_TRUE(reg1.valid(ent11));

    ASSERT_TRUE(reg1.all_of<comp_with_entity>(ent10));
    ASSERT_EQ(reg1.get<comp_with_entity>(ent10).entity, ent11);

    // Update component.
    auto ent02 = reg0.create();
    reg0.get<comp_with_entity>(ent00).entity = ent02;

    builder.create(ent02);
    builder.replace<comp_with_entity>(reg0, ent00);
    ops = builder.finish();

    ops.execute(reg1, emap, false);

    ASSERT_TRUE(emap.count(ent02));
    auto ent12 = emap.at(ent02);
    ASSERT_TRUE(reg1.valid(ent12));

    ASSERT_TRUE(reg1.all_of<comp_with_entity>(ent10));
    ASSERT_EQ(reg1.get<comp_with_entity>(ent10).entity, ent12);
}
