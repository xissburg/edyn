#ifndef EDYN_COMP_TRANSIENT_HPP
#define EDYN_COMP_TRANSIENT_HPP

#include <entt/core/fwd.hpp>
#include <entt/core/hashed_string.hpp>
#include <edyn/serialization/std_s11n.hpp>
#include <vector>

namespace edyn {

struct transient {
    std::vector<entt::id_type> ids;
};

template<typename Archive>
void serialize(Archive &archive, transient &tr) {
    archive(tr.ids);
}

}

#endif // EDYN_COMP_TRANSIENT_HPP
