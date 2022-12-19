#ifndef EDYN_SERIALIZATION_ENTT_S11N_HPP
#define EDYN_SERIALIZATION_ENTT_S11N_HPP

#include <entt/entity/fwd.hpp>
#include <entt/entity/entity.hpp>

namespace edyn {

template<typename Archive>
void serialize(Archive &archive, entt::entity &entity) {
    if constexpr(Archive::is_input::value) {
        std::underlying_type_t<entt::entity> i {};
        archive(i);
        entity = entt::entity{i};
    } else {
        auto i = entt::to_integral(entity);
        archive(i);
    }
}

}

#endif // EDYN_SERIALIZATION_ENTT_S11N_HPP
