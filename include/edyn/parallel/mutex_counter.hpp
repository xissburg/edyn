#ifndef EDYN_PARALLEL_MUTEX_COUNTER
#define EDYN_PARALLEL_MUTEX_COUNTER

#include <mutex>
#include <condition_variable>
#include "edyn/config/config.h"

namespace edyn {

class mutex_counter {
public:
    mutex_counter(size_t count = 0) 
        : m_count(count)
    {}

    auto count() {
        std::lock_guard lock(m_mutex);
        return m_count;
    }

    void increment(size_t count = 1) {
        std::lock_guard lock(m_mutex);
        m_count += count;
    }

    void decrement(size_t count = 1) {
        {
            std::lock_guard lock(m_mutex);
            EDYN_ASSERT(m_count > 0);
            EDYN_ASSERT(m_count >= count);
            m_count -= count;
        }
        m_cv.notify_one();
    }

    void wait() {
        std::unique_lock lock(m_mutex);
        m_cv.wait(lock, [&] { return m_count == 0; });
    }

private:
    size_t m_count;
    std::mutex m_mutex;
    std::condition_variable m_cv;
};

}

#endif // EDYN_PARALLEL_MUTEX_COUNTER