#ifndef EDYN_PARALLEL_PARALLEL_FOR_ASYNC_HPP
#define EDYN_PARALLEL_PARALLEL_FOR_ASYNC_HPP

#include <atomic>
#include <iterator>
#include "edyn/config/config.h"
#include "edyn/parallel/job.hpp"
#include "edyn/parallel/job_dispatcher.hpp"
#include "edyn/serialization/memory_archive.hpp"

namespace edyn {

namespace detail {

template<typename IndexType, typename Function>
struct parallel_for_async_context {
    std::atomic<IndexType> current;
    const IndexType first;
    const IndexType last;
    const IndexType step;
    const IndexType chunk_size;
    std::atomic<int> completed_counter;
    std::atomic<int> ref_counter;
    job_dispatcher *dispatcher;
    job completion;
    Function func;

    parallel_for_async_context(IndexType first, IndexType last, IndexType step,
                               IndexType chunk_size, size_t num_jobs, const job &completion,
                               job_dispatcher &dispatcher, Function func)
        : current(first)
        , first(first)
        , last(last)
        , step(step)
        , chunk_size(chunk_size)
        , completed_counter(0)
        , ref_counter(num_jobs)
        , dispatcher(&dispatcher)
        , completion(completion)
        , func(func)
    {}
};

template<typename IndexType, typename Function>
void parallel_for_async_job_func(job::data_type &data) {
    auto archive = memory_input_archive(data.data(), data.size());
    intptr_t ctx_ptr;
    archive(ctx_ptr);
    auto *ctx = reinterpret_cast<parallel_for_async_context<IndexType, Function> *>(ctx_ptr);
    const auto total = ctx->last - ctx->first;

    while (true) {
        auto begin = ctx->current.fetch_add(ctx->chunk_size, std::memory_order_relaxed);
        auto end = std::min(begin + ctx->chunk_size, ctx->last);

        if (begin >= end) {
            break;
        }

        for (size_t i = begin; i < end; i += ctx->step) {
            ctx->func(i);
        }

        const auto progress = end - begin;
        const auto completed = ctx->completed_counter.fetch_add(progress, std::memory_order_relaxed) + progress;
        EDYN_ASSERT(completed <= total);

        if (completed == total) {
            ctx->dispatcher->async(ctx->completion);
            break;
        }
    }

    auto ref_count = ctx->ref_counter.fetch_sub(1, std::memory_order_relaxed) - 1;
    EDYN_ASSERT(ref_count >= 0);

    if (ref_count == 0) {
        delete ctx;
    }
}

template<typename Iterator, typename Function>
struct parallel_for_each_async_context {
    std::atomic<size_t> current;
    const Iterator first;
    const Iterator last;
    const size_t total_size;
    const size_t chunk_size;
    std::atomic<int> completed_counter;
    std::atomic<int> ref_counter;
    job_dispatcher *dispatcher;
    job completion;
    Function func;

    parallel_for_each_async_context(Iterator first, Iterator last, size_t total_size,
                                    size_t chunk_size, size_t num_jobs, const job &completion,
                                    job_dispatcher &dispatcher, Function func)
        : current(0)
        , first(first)
        , last(last)
        , total_size(total_size)
        , chunk_size(chunk_size)
        , completed_counter(0)
        , ref_counter(num_jobs)
        , dispatcher(&dispatcher)
        , completion(completion)
        , func(func)
    {}
};

template<typename Iterator, typename Function>
void parallel_for_each_async_job_func(job::data_type &data) {
    auto archive = memory_input_archive(data.data(), data.size());
    intptr_t ctx_ptr;
    archive(ctx_ptr);
    auto *ctx = reinterpret_cast<parallel_for_each_async_context<Iterator, Function> *>(ctx_ptr);
    using value_type = typename Iterator::value_type;

    while (true) {
        auto first_index = ctx->current.fetch_add(ctx->chunk_size, std::memory_order_relaxed);

        if (first_index >= ctx->total_size) {
            break;
        }

        auto first = ctx->first;
        std::advance(first, first_index);

        auto last = first;
        std::advance(last, std::min(ctx->chunk_size, ctx->total_size - first_index));
        size_t progress = 0;

        for (; first != last; ++first, ++progress) {
            if constexpr(std::is_invocable_v<Function, value_type, size_t>) {
                ctx->func(*first, first_index + progress);
            } else {
                ctx->func(*first);
            }
        }

        auto completed = ctx->completed_counter.fetch_add(progress, std::memory_order_relaxed) + progress;
        EDYN_ASSERT(completed <= ctx->total_size);

        if (completed == ctx->total_size) {
            ctx->dispatcher->async(ctx->completion);
            break;
        }
    }

    auto ref_count = ctx->ref_counter.fetch_sub(1, std::memory_order_relaxed) - 1;
    EDYN_ASSERT(ref_count >= 0);

    if (ref_count == 0) {
        delete ctx;
    }
}

} // namespace detail

template<typename IndexType, typename Function>
void parallel_for_async(job_dispatcher &dispatcher, IndexType first, IndexType last, IndexType step, const job &completion, Function func) {
    EDYN_ASSERT(first < last);
    EDYN_ASSERT(step > IndexType{0});

    // Number of available workers.
    auto num_workers = dispatcher.num_workers();

    // Number of elements to be processed.
    auto count = last - first;

    // Size of chunk that will be processed per job iteration.
    auto count_per_worker_ceil = count / num_workers + IndexType{count % num_workers != 0};
    auto chunk_size = std::max(count_per_worker_ceil, IndexType{1});

    // Number of jobs that will be dispatched. Must not be greater than number
    // of workers.
    auto num_jobs = std::min(num_workers, count);

    // Context that's shared among all jobs. It is deallocated when the last
    // job finishes.
    using context_type = detail::parallel_for_async_context<IndexType, Function>;
    auto *context = new context_type(first, last, step, chunk_size, num_jobs, completion, dispatcher, func);

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

template<typename Iterator, typename Function>
void parallel_for_each_async(job_dispatcher &dispatcher, Iterator first, Iterator last, const job &completion, Function func) {
    EDYN_ASSERT(first != last);

    // Number of available workers.
    auto num_workers = dispatcher.num_workers();

    // Number of elements to be processed.
    auto count = static_cast<size_t>(std::distance(first, last));

    // Size of chunk that will be processed per job iteration.
    auto count_per_worker_ceil = count / num_workers + static_cast<size_t>(count % num_workers != 0);
    auto chunk_size = std::max(count_per_worker_ceil, size_t{1});

    // Number of jobs that will be dispatched. Must not be greater than number
    // of workers.
    auto num_jobs = std::min(num_workers, count);

    // Context that's shared among all jobs. It is deallocated when the last
    // job finishes.
    using context_type = detail::parallel_for_each_async_context<Iterator, Function>;
    auto *context = new context_type(first, last, count, chunk_size, num_jobs, completion, dispatcher, func);

    // Job that'll process chunks of data in worker threads.
    auto child_job = job();
    child_job.func = &detail::parallel_for_each_async_job_func<Iterator, Function>;
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
