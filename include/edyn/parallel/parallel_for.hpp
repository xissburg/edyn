#ifndef EDYN_PARALLEL_PARALLEL_FOR_HPP
#define EDYN_PARALLEL_PARALLEL_FOR_HPP

#include <mutex>
#include <atomic>
#include <numeric>
#include <iterator>
#include <forward_list>
#include "edyn/config/config.h"
#include "edyn/parallel/mutex_counter.hpp"
#include "edyn/parallel/job.hpp"
#include "edyn/parallel/job_dispatcher.hpp"

namespace edyn {

namespace detail {

/**
 * A for-loop with an atomic editable range.
 */
template<typename IndexType, typename Function>
struct ranged_for_loop {
    using atomic_index_type = std::atomic<IndexType>;

    ranged_for_loop(IndexType first, IndexType last, 
                    IndexType step, const Function *func)
        : m_first(first)
        , m_last(last)
        , m_step(step)
        , m_current(first)
        , m_func(func)
    {}

    void run() {
        auto i = m_current.load(std::memory_order_relaxed);
        while (i < m_last.load(std::memory_order_acquire)) {
            (*m_func)(i);
            i = m_current.fetch_add(m_step, std::memory_order_release) + m_step;
        }
        m_current = m_last.load();
    }

    atomic_index_type m_first;
    atomic_index_type m_last;
    atomic_index_type m_step;
    atomic_index_type m_current;
    const Function *m_func;
};

/**
 * A pool of ranged for-loops where new loops can be created by stealing a
 * range from an existing one.
 */
template<typename IndexType, typename Function>
class ranged_for_loop_pool {
public:
    using ranged_for_loop_type = ranged_for_loop<IndexType, Function>;

    ranged_for_loop_pool(IndexType first, IndexType last, 
                         IndexType step, const Function *func)
        : m_first(first)
        , m_last(last)
        , m_step(step)
        , m_func(func)
        , m_size(0)
    {}

    ranged_for_loop_type *steal() {
        // Key observation: this function is never called in parallel because
        // each `parallel_for_job` spawns another `parallel_for_job` and this 
        // function is called before a new job of this kind is scheduled.
        if (m_loops.empty()) {
            // Create the first loop with the entire range.
            auto &loop = m_loops.emplace_front(m_first, m_last, m_step, m_func);
            ++m_size;
            return &loop;
        }

        IndexType candidate_remaining = 0;
        ranged_for_loop_type *candidate_loop = nullptr;

        // Find loop with the largest number of remaining work.
        for (auto &curr_loop : m_loops) {
            auto last = curr_loop.m_last.load(std::memory_order_acquire);
            auto current = curr_loop.m_current.load(std::memory_order_acquire);
            EDYN_ASSERT(current <= last);
            auto remaining = last - current;

            if (remaining >= candidate_remaining) {
                candidate_loop = &curr_loop;
                candidate_remaining = remaining;
            }
        }

        auto last = candidate_loop->m_last.load(std::memory_order_acquire);
        auto current = candidate_loop->m_current.load(std::memory_order_acquire);

        // No work left to be stolen.
        if (!(current + 1 < last)) { 
            return nullptr;
        }

        auto remaining = last - current;
        auto first = candidate_loop->m_first.load(std::memory_order_relaxed);
        auto total = last - first;

        // Return null if the loop with the biggest amount of remaining work is
        // nearly done (i.e. the remaining work is under a percentage of the total)
        if (remaining * 100 < total * 6) {
            return nullptr;
        }

        // Effectively steal a range of work from candidate by moving its last
        // index to the middle.
        auto middle = current + remaining / 2;
        candidate_loop->m_last.store(middle, std::memory_order_release);

        auto &loop = m_loops.emplace_front(middle, last, m_step, m_func);
        ++m_size;
        return &loop;
    }

    auto size() const {
        return m_size;
    }

private:
    const IndexType m_first;
    const IndexType m_last;
    const IndexType m_step;
    const Function *m_func;
    std::forward_list<ranged_for_loop_type> m_loops;
    size_t m_size;
};

/**
 * Steals a range from the pool and runs it at time of execution.
 */
template<typename IndexType, typename Function>
class parallel_for_job: public job {
public:
    using ranged_for_loop_pool_type = ranged_for_loop_pool<IndexType, Function>;

    parallel_for_job(job_dispatcher &dispatcher,
                     std::shared_ptr<ranged_for_loop_pool_type> loop_pool,
                     std::shared_ptr<mutex_counter> counter)
        : m_dispatcher(&dispatcher)
        , m_loop_pool(loop_pool)
        , m_counter(counter)
    {}

    void run() override {
        m_counter->increment();
        auto defer = std::shared_ptr<void>(nullptr, [&] (...) { m_counter->decrement(); });

        // Only steal a range and create a new job if the total number of
        // loops is still lower than the number of workers available.
        if (m_loop_pool->size() >= m_dispatcher->num_workers()) {
            return;
        }
        
        auto *loop = m_loop_pool->steal();
        if (!loop) {
            return;
        }

        m_counter->increment();

        auto child_job = std::make_shared<parallel_for_job>(*m_dispatcher, m_loop_pool, m_counter);
        m_dispatcher->async(std::static_pointer_cast<job>(child_job));

        loop->run();
    }

private:
    job_dispatcher *m_dispatcher;
    std::shared_ptr<ranged_for_loop_pool_type> m_loop_pool;
    std::shared_ptr<mutex_counter> m_counter;
};

} // namespace detail

template<typename IndexType, typename Function>
void parallel_for(job_dispatcher &dispatcher, IndexType first, IndexType last, IndexType step, const Function &func) {
    EDYN_ASSERT(step > IndexType{0});
    using job_type = detail::parallel_for_job<IndexType, Function>;
    using loop_pool_type = detail::ranged_for_loop_pool<IndexType, Function>;

    auto loop_pool = std::make_shared<loop_pool_type>(first, last, step, &func);
    auto *loop = loop_pool->steal();
    
    auto counter = std::make_shared<mutex_counter>();
    auto child_job = std::make_shared<job_type>(dispatcher, loop_pool, counter);
    dispatcher.async(child_job);

    loop->run();
    counter->wait();
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