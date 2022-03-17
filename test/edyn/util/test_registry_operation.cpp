#include "../common/common.hpp"
#include "edyn/util/registry_operation.hpp"
#include "edyn/util/registry_operation_builder.hpp"
#include <entt/core/type_info.hpp>
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
    opc.execute(reg1, emap);

    ASSERT_TRUE(emap.contains(ent0));
    auto ent1 = emap.at(ent0);
    ASSERT_TRUE(reg1.valid(ent1));

    edyn::registry_operation opd;
    opd.operation = edyn::registry_op_type::destroy;
    opd.entities.push_back(ent0);
    // Should destroy entity in reg1 and remove it from emap.
    opd.execute(reg1, emap);

    ASSERT_FALSE(emap.contains(ent0));
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

    auto builder = edyn::registry_operation_builder_impl<comp_with_entity>{};
    auto ent0_arr = std::array{ent00, ent01};
    builder.create(ent0_arr.begin(), ent0_arr.end());
    builder.emplace<comp_with_entity>(reg0, ent00);
    auto ops = builder.finish();

    auto emap = edyn::entity_map{};
    ops.execute(reg1, emap);

    ASSERT_TRUE(emap.contains(ent00));
    auto ent10 = emap.at(ent00);
    ASSERT_TRUE(reg1.valid(ent10));

    ASSERT_TRUE(emap.contains(ent01));
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

    ops.execute(reg1, emap);

    ASSERT_TRUE(emap.contains(ent02));
    auto ent12 = emap.at(ent02);
    ASSERT_TRUE(reg1.valid(ent12));

    ASSERT_TRUE(reg1.all_of<comp_with_entity>(ent10));
    ASSERT_EQ(reg1.get<comp_with_entity>(ent10).entity, ent12);

    // Remove component.
    reg0.remove<comp_with_entity>(ent00);

    builder.remove<comp_with_entity>(reg0, ent00);
    ops = builder.finish();

    ops.execute(reg1, emap);

    ASSERT_FALSE(reg1.all_of<comp_with_entity>(ent10));
}

struct another_comp {
    double d;
};

TEST(test_registry_operation, test_impl) {
    using namespace entt::literals;
    entt::meta<comp_with_entity>().type()
        .data<&comp_with_entity::entity, entt::as_ref_t>("entity"_hs);

    auto reg0 = entt::registry{};
    auto reg1 = entt::registry{};

    auto ent00 = reg0.create();
    auto ent01 = reg0.create();
    reg0.emplace<comp_with_entity>(ent00, ent01);
    reg0.emplace<another_comp>(ent01, 1.618);

    edyn::registry_operation_builder *builder =
        new edyn::registry_operation_builder_impl<comp_with_entity, another_comp>;

    builder->create(ent00);
    builder->create(ent01);
    builder->emplace_all(reg0, ent00);
    builder->emplace_all(reg0, ent01);
    auto ops = builder->finish();

    auto emap = edyn::entity_map{};
    ops.execute(reg1, emap);

    ASSERT_TRUE(emap.contains(ent00));
    auto ent10 = emap.at(ent00);
    ASSERT_TRUE(reg1.valid(ent10));

    ASSERT_TRUE(emap.contains(ent01));
    auto ent11 = emap.at(ent01);
    ASSERT_TRUE(reg1.valid(ent11));

    ASSERT_TRUE(reg1.all_of<comp_with_entity>(ent10));
    ASSERT_EQ(reg1.get<comp_with_entity>(ent10).entity, ent11);

    ASSERT_TRUE(reg1.all_of<another_comp>(ent11));
    ASSERT_EQ(reg1.get<another_comp>(ent11).d, reg0.get<another_comp>(ent01).d);

    // Update component by id.
    reg0.get<another_comp>(ent01).d = 0.7071;

    builder->replace_type_id(reg0, ent01, entt::type_seq<another_comp>::value());
    ops = builder->finish();
    ops.execute(reg1, emap);

    ASSERT_EQ(reg1.get<another_comp>(ent11).d, reg0.get<another_comp>(ent01).d);

    // Remove component by id.
    reg0.remove<another_comp>(ent01);

    builder->remove_type_id(reg0, ent01, entt::type_seq<another_comp>::value());
    ops = builder->finish();
    ops.execute(reg1, emap);

    ASSERT_FALSE(reg1.all_of<another_comp>(ent11));
}
