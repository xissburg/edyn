#ifndef EDYN_PARALLEL_ATOMIC_COUNTER_HPP
#define EDYN_PARALLEL_ATOMIC_COUNTER_HPP

#include <atomic>
#include "edyn/config/config.h"
#include "edyn/parallel/job_dispatcher.hpp"

namespace edyn {

class atomic_counter {
public:
    void increment(unsigned int count = 1) {
        m_count.fetch_add(static_cast<int>(count), std::memory_order_relaxed);
    }

    void decrement(unsigned int count = 1) {
        auto valid = m_valid.load(std::memory_order_relaxed);
        EDYN_ASSERT(valid);
        if (!valid) return;

        auto curr_count = m_count.fetch_sub(count, std::memory_order_relaxed) - static_cast<int>(count);

        if (curr_count <= 0) {
            m_valid.store(false, std::memory_order_relaxed);
            m_dispatcher->async(m_job);
        }
    }

    job m_job;
    job_dispatcher *m_dispatcher {&job_dispatcher::global()};

private:
    std::atomic_int m_count {0};
    std::atomic_bool m_valid {true};
};

}

#endif // EDYN_PARALLEL_ATOMIC_COUNTER_HPP