#include "edyn/replication/register_external.hpp"
#include "edyn/comp/graph_node.hpp"
#include "edyn/context/registry_operation_context.hpp"

namespace edyn {

void remove_external_components(entt::registry &registry) {
    auto &settings = registry.ctx().at<edyn::settings>();
    settings.clear_actions_func = nullptr;

    auto &reg_op_ctx = registry.ctx().at<registry_operation_context>();
    reg_op_ctx.make_reg_op_builder = &make_reg_op_builder_default;
    reg_op_ctx.make_reg_op_observer = &make_reg_op_observer_default;

    if (auto *stepper = registry.ctx().find<stepper_async>()) {
        stepper->settings_changed();
        stepper->reg_op_ctx_changed();
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
