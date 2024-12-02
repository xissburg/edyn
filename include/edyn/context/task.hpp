#ifndef EDYN_CONTEXT_TASK_HPP
#define EDYN_CONTEXT_TASK_HPP

#include <cstdint>
#include <entt/entity/fwd.hpp>
#include <entt/signal/fwd.hpp>

namespace edyn {

using task_delegate_t = entt::delegate<void(unsigned start, unsigned end)>;
using task_completion_delegate_t = entt::delegate<void(void)>;
using enqueue_task_t = void (task_delegate_t task, unsigned size, task_completion_delegate_t completion);
using enqueue_task_wait_t = void (task_delegate_t task, unsigned size);

void enqueue_task_default(task_delegate_t task, unsigned size, task_completion_delegate_t completion);
void enqueue_task_wait_default(task_delegate_t task, unsigned size);

}

#endif // EDYN_CONTEXT_TASK_HPP
