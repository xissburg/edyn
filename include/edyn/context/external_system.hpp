#ifndef EDYN_PARALLEL_EXTERNAL_SYSTEM_HPP
#define EDYN_PARALLEL_EXTERNAL_SYSTEM_HPP

#include <entt/entity/fwd.hpp>

namespace edyn {

using external_system_func_t = void(*)(entt::registry &);

}

#endif // EDYN_PARALLEL_EXTERNAL_SYSTEM_HPP
