#include "../common/common.hpp"
#include "edyn/networking/networking.hpp"
#include "edyn/networking/util/client_snapshot_exporter.hpp"
#include "edyn/networking/util/client_snapshot_importer.hpp"
#include <entt/core/type_info.hpp>
#include <entt/meta/factory.hpp>
#include <entt/core/hashed_string.hpp>

struct comp {
    entt::entity entity;
    double d;
};

template<typename Archive>
void serialize(Archive &archive, comp &c) {
    archive(c.entity, c.d);
}

TEST(networking_test, client_export_import) {
    using namespace entt::literals;
    entt::meta<comp>().type()
        .data<&comp::entity, entt::as_ref_t>("entity"_hs);

    auto reg0 = entt::registry{};

    // Create networked entities.
    auto ent0 = reg0.create();
    auto ent1 = reg0.create();
    reg0.emplace<edyn::networked_tag>(ent0);
    reg0.emplace<edyn::networked_tag>(ent1);
    reg0.emplace<edyn::disabled_tag>(ent1);
    reg0.emplace<comp>(ent0, ent1, 1.618);

    // Create a non-networked entity.
    auto ent2 = reg0.create();
    reg0.emplace<comp>(ent2, ent0, 9.998);

    auto snap = edyn::registry_snapshot{};
    snap.entities.push_back(ent0);
    snap.entities.push_back(ent1);
    snap.entities.push_back(ent2);

    auto components_tuple = std::tuple_cat(edyn::networked_components, std::tuple<comp>{});
    auto exporter = edyn::client_snapshot_exporter_impl(components_tuple, {}, {});
    exporter.export_all(reg0, snap);

    auto reg1 = entt::registry{};
    auto emap = edyn::entity_map{};

    for (auto remote_entity : snap.entities) {
        if (emap.contains(remote_entity)) continue;
        auto local_entity = reg1.create();
        emap.insert(remote_entity, local_entity);
    }

    auto importer = edyn::client_snapshot_importer_impl(components_tuple);
    importer.import(reg1, emap, snap, false);

    ASSERT_TRUE((reg1.all_of<comp>(emap.at(ent0))));
    ASSERT_TRUE((reg1.all_of<edyn::disabled_tag>(emap.at(ent1))));
    // ent2 is not networked thus should not have exported its components.
    ASSERT_TRUE((!reg1.any_of<comp>(emap.at(ent2))));
    ASSERT_EQ(reg1.get<comp>(emap.at(ent0)).entity, emap.at(ent1));
    ASSERT_EQ(reg1.get<comp>(emap.at(ent0)).d, 1.618);
}
