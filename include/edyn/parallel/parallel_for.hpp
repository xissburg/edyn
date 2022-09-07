#ifndef EDYN_PARALLEL_PARALLEL_FOR_HPP
#define EDYN_PARALLEL_PARALLEL_FOR_HPP

#include <atomic>
#include <condition_variable>
#include <iterator>
#include "edyn/config/config.h"
#include "edyn/parallel/job.hpp"
#include "edyn/parallel/job_dispatcher.hpp"
#include "edyn/serialization/memory_archive.hpp"

namespace edyn {

namespace detail {

template<typename IndexType, typename Function>
struct parallel_for_context {
    std::atomic<IndexType> current;
    const IndexType last;
    const IndexType step;
    const IndexType chunk_size;
    size_t count;
    bool done;
    std::mutex mutex;
    std::condition_variable cv;
    Function func;

    parallel_for_context(IndexType first, IndexType last, IndexType step,
                         IndexType chunk_size, size_t num_jobs, Function func)
        : current(first)
        , last(last)
        , step(step)
        , chunk_size(chunk_size)
        , count(num_jobs)
        , done(false)
        , func(func)
    {}

    ~parallel_for_context() {
        EDYN_ASSERT(count == 0);
    }

    void decrement() {
        std::lock_guard lock(mutex);
        EDYN_ASSERT(count > 0);
        --count;
        done = count == 0;

        // Normally, `notify_one` would be called without the lock being held.
        // However, according to
        // https://en.cppreference.com/w/cpp/thread/condition_variable/notify_one:
        // "Notifying while under the lock may nevertheless be necessary when precise
        // scheduling of events is required, e.g. if the waiting thread would exit
        // the program if the condition is satisfied, causing destruction of the
        // notifying thread's condition_variable."
        // If the lock would be released right here, there is a very slim chance
        // that `wait()` would be called in the other thread right after, before
        // `notify_one()` below, where the lock would be acquired and the predicate
        // would test true, causing `parallel_for` to return, thus destroying this
        // object (since it is in the stack) and then everything down from here is
        // undefined behavior.
        if (done) {
            cv.notify_one();
        }
    }

    void wait() {
        std::unique_lock lock(mutex);
        cv.wait(lock, [&] { return done; });
    }
};

template<typename IndexType, typename Function>
void run_parallel_for(parallel_for_context<IndexType, Function> &ctx) {
    while (true) {
        auto begin = ctx.current.fetch_add(ctx.chunk_size, std::memory_order_relaxed);

        if (begin >= ctx.last) {
            break;
        }

        auto end = std::min(begin + ctx.chunk_size, ctx.last);

        for (auto i = begin; i < end; i += ctx.step) {
            ctx.func(i);
        }
    }
}

template<typename IndexType, typename Function>
void parallel_for_job_func(job::data_type &data) {
    auto archive = memory_input_archive(data.data(), data.size());
    intptr_t ctx_ptr;
    archive(ctx_ptr);
    auto *ctx = reinterpret_cast<parallel_for_context<IndexType, Function> *>(ctx_ptr);

    run_parallel_for(*ctx);

    ctx->decrement();
}

template<typename Iterator, typename Function>
struct parallel_for_each_context {
    std::atomic<size_t> current;
    const Iterator first;
    const Iterator last;
    const size_t total_size;
    const size_t chunk_size;
    size_t count;
    bool done;
    std::mutex mutex;
    std::condition_variable cv;
    Function func;

    parallel_for_each_context(Iterator first, Iterator last, size_t total_size,
                              size_t chunk_size, size_t num_jobs, Function func)
        : current(0)
        , first(first)
        , last(last)
        , total_size(total_size)
        , chunk_size(chunk_size)
        , count(num_jobs)
        , done(false)
        , func(func)
    {}

    ~parallel_for_each_context() {
        EDYN_ASSERT(count == 0);
    }

    void decrement() {
        std::lock_guard lock(mutex);
        EDYN_ASSERT(count > 0);
        --count;
        done = count == 0;

        if (done) {
            cv.notify_one();
        }
    }

    void wait() {
        std::unique_lock lock(mutex);
        cv.wait(lock, [&] { return done; });
    }
};

template<typename Iterator, typename Function>
void run_parallel_for_each(parallel_for_each_context<Iterator, Function> &ctx) {
    using value_type = typename Iterator::value_type;

    while (true) {
        auto first_index = ctx.current.fetch_add(ctx.chunk_size, std::memory_order_relaxed);

        if (first_index >= ctx.total_size) {
            break;
        }

        auto first = ctx.first;
        std::advance(first, first_index);

        auto last = first;
        std::advance(last, std::min(ctx.chunk_size, ctx.total_size - first_index));

        for (; first != last; ++first) {
            if constexpr(std::is_invocable_v<Function, value_type, size_t>) {
                ctx.func(*first, first_index++);
            } else {
                ctx.func(*first);
            }
        }
    }
}

template<typename Iterator, typename Function>
void parallel_for_each_job_func(job::data_type &data) {
    auto archive = memory_input_archive(data.data(), data.size());
    intptr_t ctx_ptr;
    archive(ctx_ptr);
    auto *ctx = reinterpret_cast<parallel_for_each_context<Iterator, Function> *>(ctx_ptr);

    run_parallel_for_each(*ctx);

    ctx->decrement();
}

} // namespace detail

/**
 * @brief Dynamically splits the range `[first, last)` and calls `func` in parallel
 * once for each element starting at `first` and incrementing by `step` until `last`.
 *
 * @tparam IndexType Type of the index values.
 * @tparam Function Type of function to be invoked.
 * @param dispatcher The `edyn::job_dispatcher` where the parallel jobs will be run.
 * @param first Index of the first element in the range.
 * @param last Index past the last element in the range.
 * @param step The size of each increment from `first` to `last`.
 * @param func Function that will be called for each increment of index from `first`
 * to `last` incrementing by `step`. Expected signature `void(IndexType)`.
 */
template<typename IndexType, typename Function>
void parallel_for(job_dispatcher &dispatcher, IndexType first, IndexType last, IndexType step, Function func) {
    EDYN_ASSERT(first < last);
    EDYN_ASSERT(step > IndexType{0});

    // Number of available workers.
    auto num_workers = dispatcher.num_workers();

    // Number of elements to be processed.
    auto count = last - first;
    EDYN_ASSERT(count > 1);

    // Size of chunk that will be processed per job iteration. The calling thread
    // also does work thus 1 is added to the number of workers.
    auto chunk_size = std::max(count / (num_workers + 1), IndexType{1});

    // Number of jobs that will be dispatched. Must not be greater than number
    // of workers (including this thread).
    auto num_jobs = std::min(num_workers, count - 1);

    // Context that's shared among all jobs.
    auto context = detail::parallel_for_context<IndexType, Function>(first, last, step, chunk_size, num_jobs, func);

    // Job that'll process chunks of data in worker threads.
    auto child_job = job();
    child_job.func = &detail::parallel_for_job_func<IndexType, Function>;
    auto archive = fixed_memory_output_archive(child_job.data.data(), child_job.data.size());
    auto ctx_ptr = reinterpret_cast<intptr_t>(&context);
    archive(ctx_ptr);

    // Dispatch background jobs.
    for (size_t i = 0; i < num_jobs; ++i) {
        dispatcher.async(child_job);
    }

    // Process chunks of the for loop in the current thread as well.
    detail::run_parallel_for(context);

    // Wait all background jobs to finish.
    context.wait();
}

/**
 * @brief Dynamically splits the range `[first, last)` and calls `func` in parallel
 * once for each element starting at `first` and incrementing by `step` until `last`.
 * Tasks are run in the default global `edyn::job_dispatcher`.
 *
 * @tparam IndexType Type of the index values.
 * @tparam Function Type of function to be invoked.
 * @param first Index of the first element in the range.
 * @param last Index past the last element in the range.
 * @param step The size of each increment from `first` to `last`.
 * @param func Function that will be called for each increment of index from `first`
 * to `last` incrementing by `step`. Expected signature `void(IndexType)`.
 */
template<typename IndexType, typename Function>
void parallel_for(IndexType first, IndexType last, IndexType step, Function func) {
    parallel_for(job_dispatcher::global(), first, last, step, func);
}

/**
 * @brief Dynamically splits the range `[first, last)` and calls `func` in parallel
 * once for each element` in the range. Tasks are run in the default global
 * `edyn::job_dispatcher`.
 *
 * @tparam IndexType Type of the index values.
 * @tparam Function Type of function to be invoked.
 * @param first Index of the first element in the range.
 * @param last Index past the last element in the range.
 * @param func Function that will be called for each index in `[first, last)`.
 * Expected signature `void(IndexType)`.
 */
template<typename IndexType, typename Function>
void parallel_for(IndexType first, IndexType last, Function func) {
    parallel_for(first, last, IndexType {1}, func);
}

/**
 * @brief Dynamically splits the range `[first, last)` and calls `func` in parallel
 * once for each element` in the range.
 *
 * @tparam Type of input iterator.
 * @tparam Function Type of function to be invoked.
 * @param dispatcher The `edyn::job_dispatcher` where the parallel jobs will be run.
 * @param first An iterator to the first element.
 * @param last An iterator past the last element.
 * @param func Function that will be called for each element `[first, last)`.
 * Expected signature `void(Iterator)`.
 */
template<typename Iterator, typename Function>
void parallel_for_each(job_dispatcher &dispatcher, Iterator first, Iterator last, Function func) {
    // Number of available workers.
    auto num_workers = dispatcher.num_workers();

    // Number of elements to be processed.
    auto count = std::distance(first, last);
    EDYN_ASSERT(count > 1);

    // Size of chunk that will be processed per job iteration. The calling thread
    // also does work thus 1 is added to the number of workers.
    size_t chunk_size = std::max(count / (num_workers + 1), size_t{1});

    // Number of jobs that will be dispatched. Must not be greater than number
    // of workers (including this thread).
    auto num_jobs = std::min(num_workers, size_t(count - 1));

    // Context that's shared among all jobs.
    auto context = detail::parallel_for_each_context<Iterator, Function>(first, last, count, chunk_size, num_jobs, func);

    // Job that'll process chunks of data in worker threads.
    auto child_job = job();
    child_job.func = &detail::parallel_for_each_job_func<Iterator, Function>;
    auto archive = fixed_memory_output_archive(child_job.data.data(), child_job.data.size());
    auto ctx_ptr = reinterpret_cast<intptr_t>(&context);
    archive(ctx_ptr);

    // Dispatch background jobs.
    for (size_t i = 0; i < num_jobs; ++i) {
        dispatcher.async(child_job);
    }

    // Process chunks of the for loop in the current thread as well.
    detail::run_parallel_for_each(context);

    // Wait all background jobs to finish.
    context.wait();
}

/**
 * @brief Dynamically splits the range `[first, last)` and calls `func` in parallel
 * once for each element` in the range. Tasks are run in the default global
 * `edyn::job_dispatcher`.
 *
 * @tparam Type of input iterator.
 * @tparam Function Type of function to be invoked.
 * @param first An iterator to the first element.
 * @param last An iterator past the last element.
 * @param func Function that will be called for each element `[first, last)`.
 * Expected signature `void(Iterator)`.
 */
template<typename Iterator, typename Function>
void parallel_for_each(Iterator first, Iterator last, Function func) {
    parallel_for_each(job_dispatcher::global(), first, last, func);
}

}

#endif // EDYN_PARALLEL_PARALLEL_FOR_HPP
