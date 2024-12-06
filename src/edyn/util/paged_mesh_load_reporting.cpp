#include "edyn/util/paged_mesh_load_reporting.hpp"
#include "edyn/parallel/message.hpp"
#include "edyn/parallel/message_dispatcher.hpp"
#include "edyn/shapes/paged_mesh_shape.hpp"

namespace edyn::internal {

struct paged_mesh_page_load_context {
    message_queue_handle<msg::paged_triangle_mesh_load_page> queue;
    entt::sigh<void(entt::entity, size_t)> load_signal;
};

void on_paged_triangle_mesh_load_page(entt::registry &registry, message<msg::paged_triangle_mesh_load_page> &msg) {
    auto &ctx = registry.ctx().get<paged_mesh_page_load_context>();

    for (auto [entity, shape] : registry.view<paged_mesh_shape>().each()) {
        if (shape.trimesh.get() == msg.content.trimesh) {
            ctx.load_signal.publish(entity, msg.content.mesh_index);
            break;
        }
    }
}

void init_paged_mesh_load_reporting(entt::registry &registry) {
    auto &dispatcher = message_dispatcher::global();
    auto &ctx = registry.ctx().emplace<paged_mesh_page_load_context>(dispatcher.make_queue<msg::paged_triangle_mesh_load_page>(paged_mesh_load_queue_identifier));
    ctx.queue.sink<msg::paged_triangle_mesh_load_page>().connect<&on_paged_triangle_mesh_load_page>(registry);
}

void update_paged_mesh_load_reporting(entt::registry &registry) {
    auto &ctx = registry.ctx().get<paged_mesh_page_load_context>();
    ctx.queue.update();
}

void deinit_paged_mesh_load_reporting(entt::registry &registry) {
    registry.ctx().erase<paged_mesh_page_load_context>();
}

}

namespace edyn {

entt::sink<entt::sigh<void(entt::entity, size_t)>> on_paged_mesh_page_loaded(entt::registry &registry) {
    auto &ctx = registry.ctx().get<internal::paged_mesh_page_load_context>();
    return {ctx.load_signal};
}

}
