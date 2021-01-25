#ifndef EDYN_PARALLEL_ATOMIC_COUNTER_HPP
#define EDYN_PARALLEL_ATOMIC_COUNTER_HPP

#include <atomic>
#include "edyn/config/config.h"
#include "edyn/parallel/job_dispatcher.hpp"

namespace edyn {

class atomic_counter {
public:
    atomic_counter(job &j, size_t count = 0, job_dispatcher &dispatcher = job_dispatcher::global())
        : m_job(j)
        , m_dispatcher(&dispatcher)
        , m_count(static_cast<int>(count))
    {}

    void increment(unsigned int count = 1) {
        m_count.fetch_add(static_cast<int>(count), std::memory_order_relaxed);
    }

    bool decrement(unsigned int count = 1) {
        auto curr_count = m_count.fetch_sub(count, std::memory_order_relaxed) - static_cast<int>(count);
        EDYN_ASSERT(curr_count >= 0);

        if (curr_count == 0) {
            m_dispatcher->async(m_job);
            return false;
        } else {
            return true;
        }
    }

private:
    job m_job;
    job_dispatcher *m_dispatcher;
    std::atomic_int m_count;
};

}

#endif // EDYN_PARALLEL_ATOMIC_COUNTER_HPP
