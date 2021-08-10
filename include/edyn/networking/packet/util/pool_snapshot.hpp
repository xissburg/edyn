#ifndef EDYN_NETWORKING_PACKET_POOL_SNAPSHOT_HPP
#define EDYN_NETWORKING_PACKET_POOL_SNAPSHOT_HPP

#include <array>
#include <vector>
#include <utility>
#include <entt/entity/fwd.hpp>
#include "edyn/networking/networked_comp.hpp"

namespace edyn {

struct pool_snapshot_base {
    size_t component_index;
};

template<typename Component>
struct pool_snapshot : public pool_snapshot_base {
    std::vector<std::pair<entt::entity, Component>> pairs;
};

template<typename Archive, typename Component>
void serialize(Archive &archive, pool_snapshot<Component> &pool) {
    archive(pool.pairs);
}

namespace detail {
    template<typename Archive, typename Component>
    constexpr auto make_serialize_pool_snapshot_base_function() {
        return [] (Archive &archive, pool_snapshot_base &pool) {
            auto &typed_pool = static_cast<pool_snapshot<Component> &>(pool);
            serialize(archive, typed_pool);
        };
    }

    template<typename Archive, typename... Component>
    struct serialize_pool_snapshot_base_function_array {
        using FunctionType = void(*)(Archive &, pool_snapshot_base &);
        std::array<FunctionType, sizeof...(Component)> functions;

        constexpr serialize_pool_snapshot_base_function_array()
            : functions{}
        {
            size_t i = 0;
            ((functions[i++] = make_serialize_pool_snapshot_base_function<Archive, Component>()), ...);
        }
    };

    template<typename Archive, typename... Component>
    void serialize(Archive &archive, pool_snapshot_base &pool, std::tuple<Component...>) {
        static constexpr auto func_array = detail::serialize_pool_snapshot_base_function_array<Archive, Component...>();
        func_array[pool.component_index](archive, pool);
    }
}

template<typename Archive>
void serialize(Archive &archive, pool_snapshot_base &pool) {
    detail::serialize(archive, pool, networked_components);
}

}

#endif // EDYN_NETWORKING_PACKET_POOL_SNAPSHOT_HPP
