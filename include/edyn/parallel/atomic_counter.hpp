#ifndef EDYN_PARALLEL_ATOMIC_COUNTER_HPP
#define EDYN_PARALLEL_ATOMIC_COUNTER_HPP

#include <atomic>
#include "edyn/config/config.h"
#include "edyn/parallel/job_dispatcher.hpp"
#include "edyn/serialization/memory_archive.hpp"

namespace edyn {

namespace detail {
    inline void decrement_atomic_counter_job_func(job::data_type &data);
}

class atomic_counter {
public:
    atomic_counter(const job &j, size_t count = 0, job_dispatcher &dispatcher = job_dispatcher::global())
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

    job get_decrement_job() {
        auto dec_job = job();
        dec_job.func = &detail::decrement_atomic_counter_job_func;
        auto archive = fixed_memory_output_archive(dec_job.data.data(), dec_job.data.size());
        auto ctx_ptr = reinterpret_cast<intptr_t>(this);
        archive(ctx_ptr);
        return dec_job;
    }

private:
    job m_job;
    job_dispatcher *m_dispatcher;
    std::atomic_int m_count;
};

namespace detail {
    inline void decrement_atomic_counter_job_func(job::data_type &data) {
        auto archive = memory_input_archive(data.data(), data.size());
        intptr_t ctx_ptr;
        archive(ctx_ptr);
        auto *counter = reinterpret_cast<atomic_counter *>(ctx_ptr);
        counter->decrement();
    }
}

}

#endif // EDYN_PARALLEL_ATOMIC_COUNTER_HPP
