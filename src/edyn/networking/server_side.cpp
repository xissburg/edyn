#include "edyn/networking/server_side.hpp"
#include "edyn/networking/packet/transient_snapshot.hpp"
#include "edyn/networking/remote_client.hpp"
#include "edyn/comp/transient_comp.hpp"
#include <entt/core/type_traits.hpp>
#include <entt/entity/registry.hpp>

namespace edyn {

template<typename Component>
void insert_component_in_entity_response_element(entt::registry &registry, entt::entity entity, entity_response_element &element) {
    if (!registry.has<Component>(entity)) {
        return;
    }

    auto index = tuple_index_of<Component>(networked_components);

    if constexpr(entt::is_eto_eligible_v<Component>) {
        auto ptr = std::make_unique<entity_response_component<Component>>(entity_response_component<Component>{index});
        element.components.push_back(std::move(ptr));
    } else {
        auto &comp = registry.get<Component>(entity);
        auto ptr = std::make_unique<entity_response_component<Component>>(entity_response_component<Component>{index, comp});
        element.components.push_back(std::move(ptr));
    }
}

static void process_packet(entt::registry &registry, entt::entity client_entity, const entity_request &req) {
    auto res = entity_response{};

    for (auto entity : req.entities) {
        if (!registry.valid(entity)) {
            continue;
        }

        auto &element = res.elements.emplace_back();
        element.entity = entity;

        std::apply([&] (auto ... comp) {
            ((insert_component_in_entity_response_element<decltype(comp)>(registry, entity, element)), ...);
        }, networked_components);
    }

    if (!res.elements.empty()) {
        auto &client = registry.get<remote_client>(client_entity);
        client.packet_signal.publish(edyn_packet{std::move(res)});
    }
}

static void process_packet(entt::registry &registry, entt::entity client_entity, const entity_response &res) {

}

static void process_packet(entt::registry &registry, entt::entity client_entity, const transient_snapshot &snapshot) {

}

void server_process_packet(entt::registry &registry, entt::entity client_entity, const edyn_packet &packet) {
    std::visit([&] (auto &&decoded_packet) {
        process_packet(registry, client_entity, decoded_packet);
    }, packet.var);
}

template<typename Component>
void insert_pool_in_snapshot(entt::registry &registry, transient_snapshot &snapshot) {
    auto pool = std::make_unique<pool_snapshot<Component>>();
    auto view = registry.view<Component>();

    for (auto entity : view) {
        if constexpr(entt::is_eto_eligible_v<Component>) {
            pool->pairs.emplace_back(entity);
        } else {
            auto &comp = view.get(entity);
            pool->pairs.emplace_back(entity, comp);
        }
    }

    snapshot.pools.push_back(std::move(pool));
}

transient_snapshot server_get_transient_snapshot(entt::registry &registry) {
    auto snapshot = transient_snapshot{};

    std::apply([&] (auto ... comp) {
        ((insert_pool_in_snapshot<decltype(comp)>(registry, snapshot)), ...);
    }, transient_components);

    return snapshot;
}

}
