#ifndef EDYN_CONTEXT_TASK_UTIL_HPP
#define EDYN_CONTEXT_TASK_UTIL_HPP

#include "edyn/context/settings.hpp"
#include "edyn/context/task.hpp"
#include <entt/entity/registry.hpp>
#include <functional>

namespace edyn {

inline void enqueue_task(entt::registry &registry, task_delegate_t task, unsigned size, task_completion_delegate_t completion) {
    auto &settings = registry.ctx().get<edyn::settings>();
    (*settings.enqueue_task)(task, size, completion);
}

inline void enqueue_task_wait(entt::registry &registry, task_delegate_t task, unsigned size) {
    auto &settings = registry.ctx().get<edyn::settings>();
    (*settings.enqueue_task_wait)(task, size);
}

}

#endif // EDYN_CONTEXT_TASK_UTIL_HPP
