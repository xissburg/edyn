#include "edyn/context/start_thread.hpp"
#include <thread>

namespace edyn {

void start_thread_func_default(void (*func)(void *), void *data) {
    std::thread(func, data).detach();
}

}
