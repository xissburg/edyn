#ifndef EDYN_PARALLEL_PARALLEL_FOR_HPP
#define EDYN_PARALLEL_PARALLEL_FOR_HPP

#include <mutex>
#include <atomic>
#include <future>
#include <numeric>
#include <iterator>
#include "edyn/config/config.h"
#include "edyn/parallel/job.hpp"
#include "edyn/parallel/job_dispatcher.hpp"

namespace edyn {

namespace detail {

/**
 * Job class used in `parallel_for`. It creates a chain of jobs of the same type.
 * At the time of execution, it steals a range of the remaining work of a parent.
 */
template<typename IndexType, typename Function>
class parallel_for_job: public job {
public:
    using atomic_index_type = std::atomic<IndexType>;
    using job_type = parallel_for_job<IndexType, Function>;

    parallel_for_job(job_dispatcher &dispatcher, 
                     job_type *parent, 
                     IndexType first, IndexType last, 
                     IndexType step, const Function *func)
        : m_dispatcher(&dispatcher)
        , m_parent(parent)
        , m_first(first)
        , m_last(last)
        , m_step(step)
        , m_current(first)
        , m_func(func)
    {}

    void run() override {
        // Target job to steal a range from.
        job_type *target_job = nullptr;
        IndexType target_first = 0;
        IndexType target_last = 0;
        IndexType target_remaining = 0;
        IndexType target_total = 0;

        size_t num_jobs = 0;

        // Look for parent job with the biggest number of work remaining.
        auto parent = this;

        while ((parent = parent->m_parent)) {
            ++num_jobs;
            auto last = parent->m_last.load();
            auto current = parent->m_current.load();
            auto remaining = last - current;

            if (remaining > target_remaining) {
                target_job = parent;
                target_last = last;
                target_first = current + remaining / 2; // Steal half.
                target_remaining = remaining;
                auto first = parent->m_first.load();
                target_total = last - first;
            }
        }

        if (target_job) {
            // If the parent job with the biggest amount of work remaining is
            // nearly done, do nothing and return.
            if (target_remaining * 6 < target_total) {
                m_promise.set_value();
                return;
            }

            m_first = target_first;
            m_last = target_last;
            m_current = target_first;
            // Effectively steal a range of work from parent by moving its last
            // index to the left.
            target_job->m_last = target_first;
        }

        // Only create child job if the total number of jobs is still lower
        // than the number of workers available.
        if (num_jobs < m_dispatcher->num_workers()) {
            // Child jobs don't need to start with a range, it will be
            // stolen from a parent when it runs. This ensures no attempt
            // will be made to steal a range from this child job.
            m_child = std::make_shared<job_type>(*m_dispatcher, this, 0, 0, m_step, m_func);
            m_dispatcher->async(m_child);
        }

        for (auto i = m_first.load(); i < m_last.load(); i += m_step) {
            m_current = i;
            (*m_func)(i);
        }

        m_promise.set_value();
    }

    void join() {
        auto j = this;
        while ((j = j->m_child.get())) {
            j->m_promise.get_future().get();
        }
    }

private:
    atomic_index_type m_first;
    atomic_index_type m_last;
    atomic_index_type m_step;
    atomic_index_type m_current;
    const Function *m_func;
    parallel_for_job *m_parent;
    std::shared_ptr<job_type> m_child;
    job_dispatcher *m_dispatcher;
    std::promise<void> m_promise;
};

} // namespace detail

template<typename IndexType, typename Function>
void parallel_for(job_dispatcher &dispatcher, IndexType first, IndexType last, IndexType step, const Function &func) {
    EDYN_ASSERT(step > IndexType{0});

    using job_type = detail::parallel_for_job<IndexType, Function>;

    auto root_job = job_type(dispatcher, nullptr, first, last, step, &func);
    root_job.run();
    root_job.join();
}

template<typename IndexType, typename Function>
void parallel_for(IndexType first, IndexType last, IndexType step, const Function &func) {
    parallel_for(job_dispatcher::global(), first, last, step, func);
}

template<typename IndexType, typename Function>
void parallel_for(IndexType first, IndexType last, const Function &func) {
    parallel_for(first, last, IndexType {1}, func);
}

template<typename Iterator, typename Function>
void parallel_for_each(job_dispatcher &dispatcher, Iterator first, Iterator last, const Function &func) {
    auto count = std::distance(first, last);

    parallel_for(dispatcher, size_t{0}, size_t{count}, size_t{1}, [&] (size_t index) {
        func(first + index);
    });
}

template<typename Iterator, typename Function>
void parallel_for_each(Iterator first, Iterator last, const Function &func) {
    parallel_for_each(job_dispatcher::global(), first, last, func);
}

}

#endif // EDYN_PARALLEL_PARALLEL_FOR_HPP