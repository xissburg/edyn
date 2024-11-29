#ifndef EDYN_CONTEXT_START_THREAD_HPP
#define EDYN_CONTEXT_START_THREAD_HPP

namespace edyn {

using start_thread_func_t = void(void(*)(void *), void *);

void start_thread_func_default(void (*func)(void *), void *data);

}

#endif // EDYN_CONTEXT_START_THREAD_HPP
