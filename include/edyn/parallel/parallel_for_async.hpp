#ifndef EDYN_PARALLEL_PARALLEL_FOR_ASYNC_HPP
#define EDYN_PARALLEL_PARALLEL_FOR_ASYNC_HPP

#include <atomic>
#include "edyn/config/config.h"
#include "edyn/parallel/atomic_counter.hpp"
#include "edyn/parallel/job.hpp"
#include "edyn/parallel/job_dispatcher.hpp"
#include "edyn/serialization/memory_archive.hpp"

namespace edyn {

namespace detail {

template<typename IndexType, typename Function>
struct parallel_for_async_context {
    std::atomic<IndexType> current;
    const IndexType last;
    const IndexType step;
    const IndexType chunk_size;
    atomic_counter counter;
    Function func;

    parallel_for_async_context(IndexType first, IndexType last, IndexType step, 
                               IndexType chunk_size, size_t num_jobs, job &completion, 
                               job_dispatcher &dispatcher, Function func) 
        : current(first)
        , last(last)
        , step(step)
        , chunk_size(chunk_size)
        , counter(completion, num_jobs, dispatcher)
        , func(func)
    {}
};

template<typename IndexType, typename Function>
void parallel_for_async_job_func(job::data_type &data) {
    auto archive = memory_input_archive(data.data(), data.size());
    intptr_t ctx_ptr;
    archive(ctx_ptr);
    auto *ctx = reinterpret_cast<parallel_for_async_context<IndexType, Function> *>(ctx_ptr);

    while (true) {
        auto begin = ctx->current.fetch_add(ctx->chunk_size, std::memory_order_relaxed);

        if (begin >= ctx->last) {
            break;
        }

        auto end = std::min(begin + ctx->chunk_size, ctx->last);

        for (size_t i = begin; i < end; i += ctx->step) {
            ctx->func(i);
        }
    }

    ctx->counter.decrement();

    if (!ctx->counter.valid()) {
        delete ctx;
    }
}

} // namespace detail

template<typename IndexType, typename Function>
void parallel_for_async(job_dispatcher &dispatcher, IndexType first, IndexType last, IndexType step, job &completion, Function func) {
    EDYN_ASSERT(first < last);
    EDYN_ASSERT(step > IndexType{0});

    // Number of available workers.
    auto num_workers = dispatcher.num_workers();

    // Number of elements to be processed.
    auto count = last - first;

    // Size of chunk that will be processed per job iteration.
    auto chunk_size = std::max(count / num_workers, IndexType{1});

    // Number of jobs that will be dispatched. Must not be greater than number
    // of workers.
    auto num_jobs = std::min(num_workers, count);

    // Context that's shared among all jobs. It is deallocated when the last
    // job finishes.
    auto *context = new detail::parallel_for_async_context<IndexType, Function>(first, last, step, chunk_size, num_jobs, completion, dispatcher, func);

    // Job that'll process chunks of data in worker threads.
    auto child_job = job();
    child_job.func = &detail::parallel_for_async_job_func<IndexType, Function>;
    auto archive = fixed_memory_output_archive(child_job.data.data(), child_job.data.size());
    auto ctx_ptr = reinterpret_cast<intptr_t>(context);
    archive(ctx_ptr);

    // Dispatch background jobs and return immediately after.
    for (size_t i = 0; i < num_jobs; ++i) {
        dispatcher.async(child_job);
    }
}

}

#endif // EDYN_PARALLEL_PARALLEL_FOR_ASYNC_HPP
