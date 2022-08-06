#include "edyn/replication/register_external.hpp"
#include "edyn/comp/graph_node.hpp"

namespace edyn {

void remove_external_components(entt::registry &registry) {
    auto &settings = registry.ctx().at<edyn::settings>();
    settings.make_reg_op_builder = &make_reg_op_builder_default;
    settings.make_reg_op_observer = &make_reg_op_observer_default;
    settings.index_source.reset(new component_index_source_impl(shared_components_t{}));
    settings.clear_actions_func = nullptr;

    if (auto *coordinator = registry.ctx().find<island_coordinator>()) {
        coordinator->settings_changed();
    }
}

void set_external_system_init(entt::registry &registry, external_system_func_t func) {
    registry.ctx().at<settings>().external_system_init = func;

    if (auto *coordinator = registry.ctx().find<island_coordinator>()) {
        coordinator->settings_changed();
    }
}

void set_external_system_pre_step(entt::registry &registry, external_system_func_t func) {
    registry.ctx().at<settings>().external_system_pre_step = func;

    if (auto *coordinator = registry.ctx().find<island_coordinator>()) {
        coordinator->settings_changed();
    }
}

void set_external_system_post_step(entt::registry &registry, external_system_func_t func) {
    registry.ctx().at<settings>().external_system_post_step = func;

    if (auto *coordinator = registry.ctx().find<island_coordinator>()) {
        coordinator->settings_changed();
    }
}

void set_external_system_functions(entt::registry &registry,
                                   external_system_func_t init_func,
                                   external_system_func_t pre_step_func,
                                   external_system_func_t post_step_func) {
    auto &settings = registry.ctx().at<edyn::settings>();
    settings.external_system_init = init_func;
    settings.external_system_pre_step = pre_step_func;
    settings.external_system_post_step = post_step_func;

    if (auto *coordinator = registry.ctx().find<island_coordinator>()) {
        coordinator->settings_changed();
    }
}

void remove_external_systems(entt::registry &registry) {
    auto &settings = registry.ctx().at<edyn::settings>();
    settings.external_system_init = nullptr;
    settings.external_system_pre_step = nullptr;
    settings.external_system_post_step = nullptr;

    if (auto *coordinator = registry.ctx().find<island_coordinator>()) {
        coordinator->settings_changed();
    }
}

void tag_external_entity(entt::registry &registry, entt::entity entity, bool procedural) {
    if (procedural) {
        registry.emplace<edyn::procedural_tag>(entity);
    }

    registry.emplace<edyn::external_tag>(entity);
    auto non_connecting = !procedural;
    auto node_index = registry.ctx().at<edyn::entity_graph>().insert_node(entity, non_connecting);
    registry.emplace<edyn::graph_node>(entity, node_index);
}

}
