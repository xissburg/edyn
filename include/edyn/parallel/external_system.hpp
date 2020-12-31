#ifndef EDYN_PARALLEL_EXTERNAL_SYSTEM_HPP
#define EDYN_PARALLEL_EXTERNAL_SYSTEM_HPP

#include <entt/fwd.hpp>

namespace edyn {

using external_system_func_t = void(*)(entt::registry &);

extern external_system_func_t g_external_system_init;
extern external_system_func_t g_external_system_pre_step;
extern external_system_func_t g_external_system_post_step;

void set_external_system_init(external_system_func_t func);
void set_external_system_pre_step(external_system_func_t func);
void set_external_system_post_step(external_system_func_t func);

}

#endif // EDYN_PARALLEL_EXTERNAL_SYSTEM_HPP
