#ifndef EDYN_CONTEXT_TASK_HPP
#define EDYN_CONTEXT_TASK_HPP

#include <cstdint>
#include <entt/entity/fwd.hpp>

namespace edyn {

using task_func_t = void (*)(void *ctx, unsigned start, unsigned size, unsigned thread_idx);
using task_completion_func_t = void (*)(void *ctx);
using enqueue_task_t = intptr_t (*)(task_func_t task, unsigned size, void *task_ctx, void *user_ctx, task_completion_func_t completion);
using wait_task_t = void (*)(intptr_t, void *user_ctx);

void set_task_system(entt::registry &registry, enqueue_task_t enqueue, wait_task_t wait);
void reset_task_system(entt::registry &registry);

}

#endif // EDYN_CONTEXT_TASK_HPP
