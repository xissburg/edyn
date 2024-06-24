#ifndef EDYN_CONTEXT_TASK_UTIL_HPP
#define EDYN_CONTEXT_TASK_UTIL_HPP

#include "edyn/context/settings.hpp"
#include <entt/entity/registry.hpp>
#include <functional>

namespace edyn {

namespace internal {
    template<typename T, auto Func>
    void typed_task_func(void *ctx, unsigned start, unsigned size, unsigned thread_idx) {
        auto *typed_ctx = reinterpret_cast<T *>(ctx);
        std::invoke(Func, typed_ctx, start, size, thread_idx);
    }
}

template<typename Func>
intptr_t enqueue_task(entt::registry &registry, Func func, unsigned size) {
    auto &settings = registry.ctx().at<edyn::settings>();
    return (*settings.enqueue_task)(func, size, nullptr, settings.user_task_context, nullptr);
}

template<typename Func>
void enqueue_and_wait_task(entt::registry &registry, Func func, unsigned size) {
    auto &settings = registry.ctx().at<edyn::settings>();
    auto task = (*settings.enqueue_task)(func, size, nullptr, settings.user_task_context, nullptr);
    (*settings.wait_task)(task, settings.user_task_context);
}

template<typename T, typename Func>
intptr_t enqueue_task(entt::registry &registry, T &ctx, Func T::* func, unsigned size) {
    auto &settings = registry.ctx().at<edyn::settings>();
    return (*settings.enqueue_task)(&internal::typed_task_func<T, Func>, size, &ctx, settings.user_task_context, nullptr);
}

inline void wait_task(entt::registry &registry, intptr_t task) {
    auto &settings = registry.ctx().at<edyn::settings>();
    (*settings.wait_task)(task, settings.user_task_context);
}

/*
template<typename T, typename It>
struct typed_task_it_ctx {
    T *self;
    It first, last;
};

template<typename T, typename Func, typename It>
void typed_task_func_it(void *ctx, unsigned start, unsigned size, unsigned thread_idx) {
    auto *typed_ctx = reinterpret_cast<typed_task_it_ctx<T, It> *>(ctx);
    auto first = typed_ctx->first;
    std::advance(first, start);
    auto last = first;
    std::advance(last, size);

    for (; first != last; ++first) {
        std::invoke(Func{}, typed_ctx, *first, thread_idx);
    }
}

template<typename T, typename Func, typename It>
intptr_t enqueue_task(entt::registry &registry, T &ctx, Func T::* func, It first, It last) {
    auto size = std::distance(first, last);
    auto &settings = registry.ctx().at<edyn::settings>();
    auto *typed_ctx = new typed_task_it_ctx<T, It>{&ctx, first, last};
    auto delete_ctx = [](void *ctx) {
        delete reinterpret_cast<typed_task_it_ctx<T, It> *>(ctx);
    };
    (*settings.enqueue_task)(&typed_task_func_it<T, Func, It>, size, typed_ctx, settings.user_task_context, delete_ctx);
} */

}

#endif // EDYN_CONTEXT_TASK_UTIL_HPP
