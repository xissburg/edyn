#ifndef EDYN_PARALLEL_MUTEX_COUNTER
#define EDYN_PARALLEL_MUTEX_COUNTER

#include <mutex>
#include <condition_variable>

namespace edyn {

/**
 * Counter that uses a mutex internally and thus can be waited upon.
 */
class mutex_counter {
public:
    mutex_counter & operator+=(size_t i) {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_count += i;
        return *this;
    }

    mutex_counter & operator-=(size_t i) {
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            EDYN_ASSERT(i <= m_count);
            m_count -= i;
        }
        m_cv.notify_one();
        return *this;
    }

    void increment() {
        this->operator+=(1);
    }

    void decrement() {
        this->operator-=(1);
    }

    void wait() {
        std::unique_lock<std::mutex> lock(m_mutex);
        m_cv.wait(lock, [&] { return m_count == 0; });
    }

private:
    size_t m_count {0};
    std::mutex m_mutex;
    std::condition_variable m_cv;
};

}

#endif // EDYN_PARALLEL_MUTEX_COUNTER