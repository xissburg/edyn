#ifndef EDYN_NETWORKING_PACKET_POOL_SNAPSHOT_HPP
#define EDYN_NETWORKING_PACKET_POOL_SNAPSHOT_HPP

#include <memory>
#include <tuple>
#include <vector>
#include <entt/entity/fwd.hpp>
#include "edyn/networking/util/pool_snapshot_data.hpp"

namespace edyn {

struct pool_snapshot {
    unsigned component_index;
    std::shared_ptr<pool_snapshot_data> ptr;
};

extern std::unique_ptr<pool_snapshot_data>(*g_make_pool_snapshot_data)(unsigned);

template<typename Archive>
void serialize(Archive &archive, pool_snapshot &pool) {
    archive(pool.component_index);
    std::vector<uint8_t> data;

    if constexpr(Archive::is_input::value) {
        archive(data);
        auto input = memory_input_archive(data.data(), data.size());
        pool.ptr = (*g_make_pool_snapshot_data)(pool.component_index);
        pool.ptr->read(input);
    } else {
        auto output = memory_output_archive(data);
        pool.ptr->write(output);
        archive(data);
    }
}

template<typename... Components>
auto create_make_pool_snapshot_data_function([[maybe_unused]] std::tuple<Components...>) {
    return [](unsigned component_index) {
        std::tuple<Components...> components;
        auto ptr = std::unique_ptr<pool_snapshot_data>{};
        visit_tuple(components, component_index, [&](auto &&c) {
            using CompType = std::decay_t<decltype(c)>;
            ptr.reset(new pool_snapshot_data_impl<CompType>);
        });
        return ptr;
    };
}

}

#endif // EDYN_NETWORKING_PACKET_POOL_SNAPSHOT_HPP
