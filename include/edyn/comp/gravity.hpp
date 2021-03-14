#ifndef EDYN_COMP_GRAVITY_HPP
#define EDYN_COMP_GRAVITY_HPP

#include <array>
#include <entt/fwd.hpp>
#include <entt/entity/entity.hpp>

namespace edyn {

struct gravity {
    std::array<entt::entity, 2> body {entt::null, entt::null};
};

}

#endif // EDYN_COMP_GRAVITY_HPP