#include "edyn/networking/client_side.hpp"
#include "edyn/networking/packet/entity_request.hpp"

namespace edyn {

static void process_packet(entt::registry &registry, const entity_request &req) {

}

static void process_packet(entt::registry &registry, const entity_response &res) {

}

template<typename Component>
void import_pool(entt::registry &registry, const pool_snapshot<Component> &pool) {
    if constexpr(entt::is_eto_eligible_v<Component>) {
        return;
    }

    for (auto &pair : pool.pairs) {
        registry.replace<Component>(pair.first, pair.second);
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