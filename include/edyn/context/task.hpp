#ifndef EDYN_CONTEXT_TASK_HPP
#define EDYN_CONTEXT_TASK_HPP

#include <cstdint>
#include <entt/entity/fwd.hpp>
#include <entt/signal/fwd.hpp>

namespace edyn {

// EnTT delegate for background tasks. It receives a range as argument.
using task_delegate_t = entt::delegate<void(unsigned start, unsigned end)>;
// EnTT delegate to be called when a background task is finished.
using task_completion_delegate_t = entt::delegate<void(void)>;
// Function to schedule a task to be run in worker threads and return immediately.
// It takes a task delegate to be invoked over the range [0, size) and an
// optional completion to be called when the task is done.
using enqueue_task_t = void (task_delegate_t task, unsigned size, task_completion_delegate_t completion);
// Function to schedule a task to be run in worker threads and wait for
// execution to be finished.
using enqueue_task_wait_t = void (task_delegate_t task, unsigned size);

void enqueue_task_default(task_delegate_t task, unsigned size, task_completion_delegate_t completion);
void enqueue_task_wait_default(task_delegate_t task, unsigned size);

}

#endif // EDYN_CONTEXT_TASK_HPP
