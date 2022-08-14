#ifndef EDYN_REPLICATION_REGISTER_EXTERNAL_HPP
#define EDYN_REPLICATION_REGISTER_EXTERNAL_HPP

#include <entt/entity/registry.hpp>
#include "edyn/comp/action_list.hpp"
#include "edyn/comp/shared_comp.hpp"
#include "edyn/context/settings.hpp"
#include "edyn/context/registry_operation_context.hpp"
#include "edyn/replication/component_index_source.hpp"
#include "edyn/replication/registry_operation_builder.hpp"
#include "edyn/replication/registry_operation_observer.hpp"
#include "edyn/simulation/stepper_async.hpp"

namespace edyn {

/**
 * @brief Registers external components to be shared with the asynchronous
 * simulation worker.
 * @tparam Components External component types.
 * @tparam Actions All action types. Note that **actions are not components**.
 * Instead, they're stored in `edyn::action_list<Action>` which is an actual
 * component that is assigned to an entity and contains a list of actions.
 * @param registry Data source.
 * @param actions Optional tuple of actions.
 */
template<typename... Components, typename... Actions>
void register_external_components(entt::registry &registry, std::tuple<Actions...> actions = {}) {
    auto &reg_op_ctx = registry.ctx().at<registry_operation_context>();
    reg_op_ctx.make_reg_op_builder = [](entt::registry &registry) {
        auto external = std::tuple<Components...>{};
        auto action_lists = std::tuple<action_list<Actions>...>{};
        auto all_components = std::tuple_cat(shared_components_t{}, external, action_lists);
        return std::unique_ptr<registry_operation_builder>(
            new registry_operation_builder_impl(registry, all_components));
    };
    reg_op_ctx.make_reg_op_observer = [](registry_operation_builder &builder) {
        auto external = std::tuple<Components...>{};
        auto action_lists = std::tuple<action_list<Actions>...>{};
        auto all_components = std::tuple_cat(shared_components_t{}, external, action_lists);
        return std::unique_ptr<registry_operation_observer>(
            new registry_operation_observer_impl(builder, all_components));
    };

    auto external = std::tuple<Components...>{};
    auto action_lists = std::tuple<action_list<Actions>...>{};
    auto all_components = std::tuple_cat(shared_components_t{}, external, action_lists);

    auto &settings = registry.ctx().at<edyn::settings>();
    settings.index_source.reset(new component_index_source_impl(all_components));

    if constexpr(sizeof...(Actions) > 0) {
        settings.clear_actions_func = [](entt::registry &registry) {
            (registry.clear<action_list<Actions>>(), ...);
        };
    }

    if (auto *stepper = registry.ctx().find<stepper_async>()) {
        stepper->settings_changed();
        stepper->reg_op_ctx_changed();
    }
}

template<typename... Component, typename... Actions>
void register_external_components(entt::registry &registry,
                                  [[maybe_unused]] std::tuple<Component...>,
                                  std::tuple<Actions...> actions = {}) {
    register_external_components<Component...>(registry, actions);
}

/**
 * @brief Removes registered external components and resets to defaults.
 */
void remove_external_components(entt::registry &registry);

/**
 * @brief Assigns a function to be called once after a new island worker is
 * created and initialized in a worker thread.
 * @param registry Data source.
 * @param func The function.
 */
void set_external_system_init(entt::registry &registry, external_system_func_t func);

/**
 * @brief Assigns a function to be called before each simulation step in each
 * island worker in a worker thread.
 * @param registry Data source.
 * @param func The function.
 */
void set_external_system_pre_step(entt::registry &registry, external_system_func_t func);

/**
 * @brief Assigns a function to be called after each simulation step in each
 * island worker in a worker thread.
 * @param registry Data source.
 * @param func The function.
 */
void set_external_system_post_step(entt::registry &registry, external_system_func_t func);

/**
 * @brief Set all external system functions in one call. See individual setters
 * for details.
 * @param registry Data source.
 * @param init_func Initialization function called on island worker start up.
 * @param pre_step_func Called before each step.
 * @param post_step_func Called after each step.
 */
void set_external_system_functions(entt::registry &registry,
                                   external_system_func_t init_func,
                                   external_system_func_t pre_step_func,
                                   external_system_func_t post_step_func);

/**
 * @brief Remove all external system functions.
 * @param registry Data source.
 */
void remove_external_systems(entt::registry &registry);

/**
 * @brief Assigns an `external_tag` to this entity and inserts it as a node
 * into the entity graph.
 * This makes it possible to tie this entity and its components to another
 * node such as a rigid body, which means that it will be moved into the
 * island where the rigid body resides.
 * @param registry Data source.
 * @param entity The entity to be tagged.
 * @param procedural If true, the entity will reside exclusively in one island.
 */
void tag_external_entity(entt::registry &registry, entt::entity entity, bool procedural = true);

}

#endif // EDYN_REPLICATION_REGISTER_EXTERNAL_HPP
