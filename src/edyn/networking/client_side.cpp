#include "edyn/networking/client_side.hpp"
#include "edyn/networking/packet/entity_request.hpp"
#include "edyn/networking/client_networking_context.hpp"
#include <entt/entity/registry.hpp>

namespace edyn {

static void process_packet(entt::registry &registry, const entity_request &req) {

}

template<typename Component>
static void emplace_component(entt::registry &registry, entt::entity entity, const entity_response_component_base &comp) {
    auto &typed_comp = static_cast<const entity_response_component<Component> &>(comp);
    registry.emplace<Component>(entity, typed_comp.value);
}

template<typename... Component>
static void assign_component(entt::registry &registry, entt::entity entity, const entity_response_component_base &comp, std::tuple<Component...>) {
    size_t i = 0;
    ((i++ == comp.component_index ? emplace_component<Component>(registry, entity, comp) : void(0)), ...);
}

static void process_packet(entt::registry &registry, const entity_response &res) {
    auto &ctx = registry.ctx<client_networking_context>();

    for (auto &element : res.elements) {
        auto local_entity = registry.create();
        ctx.entity_map.insert(element.entity, local_entity);

        for (auto &comp_ptr : element.components) {
            assign_component(registry, local_entity, *comp_ptr, networked_components);
        }
    }
}

template<typename Component>
void import_pool(entt::registry &registry, const pool_snapshot<Component> &pool) {
    if constexpr(entt::is_eto_eligible_v<Component>) {
        return;
    }

    auto &ctx = registry.ctx<client_networking_context>();

    for (auto &pair : pool.pairs) {
        auto remote_entity = pair.first;

        if (ctx.entity_map.has_rem(pair.first)) {
            auto local_entity = ctx.entity_map.remloc(remote_entity);
            registry.replace<Component>(local_entity, pair.second);
        } else {
            ctx.request_entity_signal.publish(remote_entity);
        }
    }
}

void process_pool(entt::registry &registry, const pool_snapshot_base &pool) {
    std::apply([&] (auto ... comp) {
        size_t i = 0;
        ((i++ == pool.component_index ? import_pool(registry, static_cast<const pool_snapshot<decltype(comp)> &>(pool)) : void(0)), ...);
    }, networked_components);
}

static void process_packet(entt::registry &registry, const transient_snapshot &snapshot) {
    for (auto &pool_ptr : snapshot.pools) {
        process_pool(registry, *pool_ptr);
    }
}

void client_process_packet(entt::registry &registry, const edyn_packet &packet) {
    std::visit([&] (auto &&inner_packet) {
        process_packet(registry, inner_packet);
    }, packet.var);
}

}
