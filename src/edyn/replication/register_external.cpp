#include "edyn/replication/register_external.hpp"
#include "edyn/comp/child_list.hpp"
#include "edyn/comp/graph_node.hpp"
#include "edyn/context/registry_operation_context.hpp"
#include "edyn/networking/context/client_network_context.hpp"

namespace edyn {

void remove_external_components(entt::registry &registry) {
    auto &settings = registry.ctx().get<edyn::settings>();
    settings.clear_actions_func = nullptr;

    auto &reg_op_ctx = registry.ctx().get<registry_operation_context>();
    reg_op_ctx.make_reg_op_builder = &make_reg_op_builder_default;
    reg_op_ctx.make_reg_op_observer = &make_reg_op_observer_default;

    if (auto *stepper = registry.ctx().find<stepper_async>()) {
        stepper->settings_changed();
        stepper->reg_op_ctx_changed();
    }

    if (auto *ctx = registry.ctx().find<client_network_context>()) {
        ctx->extrapolator->set_settings(settings);
        ctx->extrapolator->set_registry_operation_context(reg_op_ctx);
    }
}

void tag_external_entity(entt::registry &registry, entt::entity entity, bool procedural) {
    if (procedural) {
        registry.emplace<procedural_tag>(entity);
    }

    registry.emplace<external_tag>(entity);
    auto &graph = registry.ctx().get<entity_graph>();
    auto non_connecting = !procedural;
    auto node_index = graph.insert_node(entity, non_connecting);
    registry.emplace<graph_node>(entity, node_index);

    if (procedural) {
        registry.emplace<island_resident>(entity);
    } else {
        registry.emplace<multi_island_resident>(entity);
    }
}

void untag_external_entity(entt::registry &registry, entt::entity entity) {
    // The `on_destroy` observers in `stepper_async` or `stepper_seq` will
    // remove this node from the entity graph.
    registry.erase<external_tag, graph_node>(entity);
    registry.remove<island_resident>(entity);
    registry.remove<multi_island_resident>(entity);
    registry.remove<procedural_tag>(entity);
}

void add_child(entt::registry &registry, entt::entity parent, entt::entity child) {
    EDYN_ASSERT(registry.all_of<graph_node>(parent));
    auto &par = registry.get_or_emplace<parent_comp>(parent);
    registry.emplace<child_list>(child, parent, par.child);
    par.child = child;
}

void remove_child(entt::registry &registry, entt::entity parent, entt::entity child) {
    auto &par = registry.get<parent_comp>(parent);
    auto child_view = registry.view<child_list>();
    auto next_child = child_view.get<child_list>(child).next;

    if (par.child == child) {
        par.child = next_child;
    } else {
        auto curr = par.child;
        while (curr != entt::null) {
            auto [curr_child] = child_view.get(curr);

            if (curr_child.next == child) {
                curr_child.next = next_child;
                break;
            }

            curr = curr_child.next;
        }
    }
}

}
