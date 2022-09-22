#ifndef EDYN_UTIL_ISLAND_UTIL_HPP
#define EDYN_UTIL_ISLAND_UTIL_HPP

#include <entt/entity/utility.hpp>
#include "edyn/comp/tag.hpp"

namespace edyn {

static constexpr auto exclude_sleeping_disabled = entt::exclude_t<sleeping_tag, disabled_tag>{};

}

#endif // EDYN_UTIL_ISLAND_UTIL_HPP
