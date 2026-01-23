#ifndef EDYN_COMP_TRANSIENT_HPP
#define EDYN_COMP_TRANSIENT_HPP

#include <entt/core/fwd.hpp>
#include <entt/core/hashed_string.hpp>
#include <vector>

namespace edyn {

struct transient {
    std::vector<entt::id_type> ids;
};

}

#endif // EDYN_COMP_TRANSIENT_HPP
