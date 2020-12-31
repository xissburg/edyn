#include "edyn/parallel/external_system.hpp"

namespace edyn {

external_system_func_t g_external_system_init = nullptr;
external_system_func_t g_external_system_pre_step = nullptr;
external_system_func_t g_external_system_post_step = nullptr;

void set_external_system_init(external_system_func_t func) {
    g_external_system_init = func;
}

void set_external_system_pre_step(external_system_func_t func) {
    g_external_system_pre_step = func;
}

void set_external_system_post_step(external_system_func_t func) {
    g_external_system_post_step = func;
}

}