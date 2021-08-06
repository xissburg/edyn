#include "../common/common.hpp"
#include "edyn/networking/networking.hpp"
#include "edyn/util/tuple_util.hpp"
#include <vector>
#include <cstdint>

TEST(network_packets_test, test_entity_req_res) {
    auto registry = entt::registry{};

    auto entity = registry.create();
    registry.emplace<edyn::position>(entity, .1f, .3f, -.9f);

    auto entity_req_out = edyn::entity_request{};
    entity_req_out.entities.push_back(entity);

    auto entity_res_out = edyn::entity_response{};
    auto &element = entity_res_out.elements.emplace_back();
    element.entity = entity;
    auto comp_ptr = std::make_unique<edyn::entity_response_component<edyn::position>>();
    comp_ptr->component_index = edyn::tuple_index_of<edyn::position>(edyn::networked_components);
    comp_ptr->value = registry.get<edyn::position>(entity);
    element.components.push_back(std::move(comp_ptr));

    auto buffer = std::vector<uint8_t>{};
    auto archive_out = edyn::memory_output_archive(buffer);
    archive_out(entity_req_out);
    archive_out(entity_res_out);

    auto entity_req_in = edyn::entity_request{};
    auto entity_res_in = edyn::entity_response{};

    auto archive_in = edyn::memory_input_archive(buffer.data(), buffer.size());
    archive_in(entity_req_in);
    archive_in(entity_res_in);

    ASSERT_EQ(entity_req_out.entities.front(), entity_req_in.entities.front());

    ASSERT_EQ(entity_res_out.elements.size(), entity_res_in.elements.size());
    ASSERT_EQ(entity_res_out.elements[0].entity, entity_res_in.elements[0].entity);
    ASSERT_EQ(entity_res_out.elements[0].components.size(), entity_res_in.elements[0].components.size());
    ASSERT_EQ(entity_res_out.elements[0].components[0]->component_index, entity_res_in.elements[0].components[0]->component_index);

    edyn::visit_entity_response_component(*entity_res_in.elements[0].components[0], [] (auto &&comp) {
        if constexpr(std::is_same_v<std::decay_t<decltype(comp)>, edyn::position>) {
            ASSERT_SCALAR_EQ(comp.x, .1f);
            ASSERT_SCALAR_EQ(comp.y, .3f);
            ASSERT_SCALAR_EQ(comp.z, .9f);
        }
    });
}
