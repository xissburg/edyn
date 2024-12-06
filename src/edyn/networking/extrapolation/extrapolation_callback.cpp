#include "edyn/networking/extrapolation/extrapolation_callback.hpp"
#include "edyn/context/settings.hpp"
#include "edyn/networking/settings/client_network_settings.hpp"
#include <entt/entity/registry.hpp>

namespace edyn {

template<typename Member>
void set_extrapolation_callback_member(entt::registry &registry, extrapolation_callback_t func,
                                       Member client_network_settings::*member) {
    auto &settings = registry.ctx().get<edyn::settings>();
    auto &client_settings = std::get<client_network_settings>(settings.network_settings);
    client_settings.*member = func;
    internal::notify_settings(registry, settings);
}

void set_extrapolation_init_callback(entt::registry &registry, extrapolation_callback_t func) {
    set_extrapolation_callback_member(registry, func, &client_network_settings::extrapolation_init_callback);
}

void set_extrapolation_deinit_callback(entt::registry &registry, extrapolation_callback_t func) {
    set_extrapolation_callback_member(registry, func, &client_network_settings::extrapolation_deinit_callback);
}

void set_extrapolation_begin_callback(entt::registry &registry, extrapolation_callback_t func) {
    set_extrapolation_callback_member(registry, func, &client_network_settings::extrapolation_begin_callback);
}

void set_extrapolation_finish_callback(entt::registry &registry, extrapolation_callback_t func) {
    set_extrapolation_callback_member(registry, func, &client_network_settings::extrapolation_finish_callback);
}

void set_extrapolation_pre_step_callback(entt::registry &registry, extrapolation_callback_t func) {
    set_extrapolation_callback_member(registry, func, &client_network_settings::extrapolation_pre_step_callback);
}

void set_extrapolation_post_step_callback(entt::registry &registry, extrapolation_callback_t func) {
    set_extrapolation_callback_member(registry, func, &client_network_settings::extrapolation_post_step_callback);
}

void set_extrapolation_callbacks(entt::registry &registry,
                                 extrapolation_callback_t init,
                                 extrapolation_callback_t deinit,
                                 extrapolation_callback_t begin,
                                 extrapolation_callback_t finish,
                                 extrapolation_callback_t pre_step,
                                 extrapolation_callback_t post_step) {
    auto &settings = registry.ctx().get<edyn::settings>();
    auto &client_settings = std::get<client_network_settings>(settings.network_settings);
    client_settings.extrapolation_init_callback = init;
    client_settings.extrapolation_deinit_callback = deinit;
    client_settings.extrapolation_begin_callback = begin;
    client_settings.extrapolation_finish_callback = finish;
    client_settings.extrapolation_pre_step_callback = pre_step;
    client_settings.extrapolation_post_step_callback = post_step;
    internal::notify_settings(registry, settings);
}

}
