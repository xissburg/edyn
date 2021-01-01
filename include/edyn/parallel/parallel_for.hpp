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
#include "edyn/serialization/memory_archive.hpp"

namespace edyn {

namespace detail {

/**
 * A for-loop with an atomic editable range.
 */
template<typename IndexType, typename Function>
struct for_loop_range {
    using atomic_index_type = std::atomic<IndexType>;

    for_loop_range(IndexType first, IndexType last, 
                   IndexType step, Function *func,
                   mutex_counter &counter)
        : m_first(first)
        , m_last(last)
        , m_step(step)
        , m_current(first)
        , m_func(func)
        , m_counter(&counter)
    {}

    /**
     * Executes the for-loop. Increments the atomic `m_current` on each iteration.
     * Decrements the counter given in the constructor once finished.
     */
    void run() {
        auto i = m_current.load(std::memory_order_relaxed);
        while (i < m_last.load(std::memory_order_acquire)) {
            (*m_func)(i);
            i = m_current.fetch_add(m_step, std::memory_order_release) + m_step;
        }
        m_counter->decrement();
    }

    atomic_index_type m_first;
    atomic_index_type m_last;
    atomic_index_type m_step;
    atomic_index_type m_current;
    Function *m_func;
    mutex_counter *m_counter;
};

/**
 * A pool of for-loops where new loops can be created by stealing a
 * range from another loop in the pool.
 */
template<typename IndexType, typename Function>
class for_loop_range_pool {
public:
    using for_loop_range_type = for_loop_range<IndexType, Function>;

    for_loop_range_pool(IndexType first, IndexType last, 
                        IndexType step, Function *func)
        : m_first(first)
        , m_last(last)
        , m_step(step)
        , m_func(func)
        , m_counter(1)
    {
        m_root = create(first, last);
    }

    for_loop_range_type *create(IndexType first, IndexType last) {
        // TODO: this function could be invoked in parallel, use a lock free
        // linked list for the loops.
        auto &loop = m_loops.emplace_front(first, last, m_step, m_func, m_counter);
        return &loop;
    }

    for_loop_range_type *root() {
        return m_root;
    }

    void wait() {
        m_counter.wait();
    }

    const IndexType m_first;
    const IndexType m_last;
    const IndexType m_step;
    Function *m_func;
    std::forward_list<for_loop_range_type> m_loops;
    mutex_counter m_counter;
    for_loop_range_type *m_root;
};

template<typename IndexType, typename Function>
struct parallel_for_shared_context {
    using for_loop_range_pool_type = for_loop_range_pool<IndexType, Function>;
    job_dispatcher *m_dispatcher;
    for_loop_range_pool_type m_loop_pool;
    std::atomic<int> m_num_jobs;

    parallel_for_shared_context(IndexType first, IndexType last, 
                                IndexType step, Function *func,
                                job_dispatcher &dispatcher)
        : m_loop_pool(first, last, step, func)
        , m_dispatcher(&dispatcher)
        , m_num_jobs(0)
    {}
};

template<typename IndexType, typename Function>
struct parallel_for_context {
    using for_loop_range_type = for_loop_range<IndexType, Function>;
    using shared_context_type = parallel_for_shared_context<IndexType, Function>;
    shared_context_type *m_shared_ctx;
    for_loop_range_type *m_parent;

    parallel_for_context()
        : m_shared_ctx(nullptr)
        , m_parent(nullptr)
    {}

    parallel_for_context(shared_context_type *shared_ctx,
                         for_loop_range_type *parent)
        : m_shared_ctx(shared_ctx)
        , m_parent(parent)
    {}
};

template<typename Archive, typename IndexType, typename Function>
void serialize(Archive &archive, parallel_for_context<IndexType, Function> &ctx) {
    if constexpr(Archive::is_output::value) {
        auto intptr = reinterpret_cast<intptr_t>(ctx.m_shared_ctx);
        archive(intptr);
        intptr = reinterpret_cast<intptr_t>(ctx.m_parent);
        archive(intptr);
    } else {
        intptr_t intptr;
        archive(intptr);
        ctx.m_shared_ctx = reinterpret_cast<parallel_for_shared_context<IndexType, Function> *>(intptr);
        archive(intptr);
        ctx.m_parent = reinterpret_cast<for_loop_range<IndexType, Function> *>(intptr);
    }
}

template<typename IndexType, typename Function>
void parallel_for_job_func(job::data_type &data) {
    auto archive = memory_input_archive(data.data(), data.size());
    auto ctx = parallel_for_context<IndexType, Function>();
    archive(ctx);

    auto shared_ctx = ctx.m_shared_ctx;
    auto parent = ctx.m_parent;

    // Decrement job count and if zero delete shared context on exit.
    auto defer = std::shared_ptr<void>(nullptr, [shared_ctx] (void *) { 
        auto num_jobs = shared_ctx->m_num_jobs.fetch_sub(1, std::memory_order_relaxed) - 1;
        if (num_jobs == 0) delete shared_ctx;
    });
    
    auto last = parent->m_last.load(std::memory_order_acquire);
    auto current = parent->m_current.load(std::memory_order_acquire);

    // No work left to be stolen.
    if (!(current + 1 < last)) { 
        return;
    }

    auto remaining = last - current;
    auto first = parent->m_first.load(std::memory_order_relaxed);
    auto total = last - first;

    // Return null if the loop with the biggest amount of remaining work is
    // nearly done (i.e. the remaining work is under a percentage of the total)
    if (remaining * 100 < total * 10) {
        return;
    }

    // Increment loop counter. Will be decremented by the new loop when
    // it finishes running. It is important to increment it before range
    // stealing below, since it might terminate the `candidate_loop` right
    // after and cause `m_counter` to be decremented to zero thus causing
    // a wait on the counter to return prematurely resulting in an incomplete
    // run of the entire for-loop range.
    shared_ctx->m_loop_pool.m_counter.increment();

    // Effectively steal a range of work from candidate by moving its last
    // index to the halfway point between current and last.
    auto middle = current + remaining / 2;
    parent->m_last.store(middle, std::memory_order_release);

    // It is possible that by the time `middle` is stored in `candidate_loop->m_last`,
    // `candidate_loop->m_current` is greater than `middle` since the for-loop
    // is running while this range stealing is taking place. To prevent calling
    // `m_func` more than once for the elements between `middle` and 
    // `candidate_loop->m_current`, load `candidate_loop->m_current` and check
    // if it's greater than or equals to `middle` and if so, start from there instead.
    current = parent->m_current.load(std::memory_order_acquire);
    auto new_first = current >= middle ? current : middle;

    auto *loop = shared_ctx->m_loop_pool.create(new_first, last);

    // Dispatch one child job that will split the parent job range if it is
    // executed before the parent job is nearly done.
    {
        shared_ctx->m_num_jobs.fetch_add(1, std::memory_order_relaxed);

        auto child_job = job();
        child_job.func = &parallel_for_job_func<IndexType, Function>;

        auto archive = fixed_memory_output_archive(child_job.data.data(), child_job.data.size());
        auto ctx = parallel_for_context<IndexType, Function>(shared_ctx, parent);
        archive(ctx);

        shared_ctx->m_dispatcher->async(child_job);
    }

    // Dispatch another child job which will split this range if it is executed
    // before this job is nearly done.
    {
        shared_ctx->m_num_jobs.fetch_add(1, std::memory_order_relaxed);

        auto child_job = job();
        child_job.func = &parallel_for_job_func<IndexType, Function>;

        auto archive = fixed_memory_output_archive(child_job.data.data(), child_job.data.size());
        auto ctx = parallel_for_context<IndexType, Function>(shared_ctx, loop);
        archive(ctx);

        shared_ctx->m_dispatcher->async(child_job);
    }

    loop->run();
}

} // namespace detail

template<typename IndexType, typename Function>
void parallel_for(job_dispatcher &dispatcher, IndexType first, IndexType last, IndexType step, Function func) {
    EDYN_ASSERT(step > IndexType{0});

    // The last job to run will delete `ctx`.
    auto *shared_ctx = new detail::parallel_for_shared_context<IndexType, Function>(first, last, step, &func, dispatcher);

    // Increment job count so the this execution unit is accounted for.
    shared_ctx->m_num_jobs.fetch_add(1, std::memory_order_relaxed);

    // On exit decrement job count and if zero delete `ctx`.
    auto defer = std::shared_ptr<void>(nullptr, [shared_ctx] (void *) { 
        auto num_jobs = shared_ctx->m_num_jobs.fetch_sub(1, std::memory_order_relaxed) - 1;
        if (num_jobs == 0) delete shared_ctx;
    });

    // Create child job which will steal a range of the for-loop if it gets a
    // chance to be executed.
    {
        shared_ctx->m_num_jobs.fetch_add(1, std::memory_order_relaxed);

        auto child_job = job();
        child_job.func = &detail::parallel_for_job_func<IndexType, Function>;

        auto archive = fixed_memory_output_archive(child_job.data.data(), child_job.data.size());
        auto ctx = detail::parallel_for_context<IndexType, Function>(shared_ctx, shared_ctx->m_loop_pool.root());
        archive(ctx);

        dispatcher.async(child_job);
    }

    shared_ctx->m_loop_pool.root()->run();

    // Wait for all for-loops to complete.
    shared_ctx->m_loop_pool.wait();
}

template<typename IndexType, typename Function>
void parallel_for(IndexType first, IndexType last, IndexType step, Function func) {
    parallel_for(job_dispatcher::global(), first, last, step, func);
}

template<typename IndexType, typename Function>
void parallel_for(IndexType first, IndexType last, Function func) {
    parallel_for(first, last, IndexType {1}, func);
}

template<typename Iterator, typename Function>
void parallel_for_each(job_dispatcher &dispatcher, Iterator first, Iterator last, Function func) {
    auto count = std::distance(first, last);

    parallel_for(dispatcher, size_t{0}, static_cast<size_t>(count), size_t{1}, [&] (size_t index) {
        func(first + index);
    });
}

template<typename Iterator, typename Function>
void parallel_for_each(Iterator first, Iterator last, Function func) {
    parallel_for_each(job_dispatcher::global(), first, last, func);
}

}

#endif // EDYN_PARALLEL_PARALLEL_FOR_HPP