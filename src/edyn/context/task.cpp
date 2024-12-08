#include "edyn/context/task.hpp"
#include "edyn/config/config.h"
#include "edyn/context/settings.hpp"
#include "edyn/parallel/job_dispatcher.hpp"
#include "edyn/serialization/memory_archive.hpp"
#include "edyn/util/settings_util.hpp"
#include <cstdint>
#include <entt/entity/registry.hpp>
#include <entt/signal/delegate.hpp>

namespace edyn {

struct parallel_for_context {
    using IndexType = size_t;
    std::atomic<IndexType> current;
    const IndexType last;
    const IndexType step;
    const IndexType chunk_size;
    size_t count;
    bool done;
    std::mutex mutex;
    std::condition_variable cv;
    task_delegate_t task;

    parallel_for_context(IndexType first, IndexType last, IndexType step,
                         IndexType chunk_size, size_t num_jobs, task_delegate_t task)
        : current(first)
        , last(last)
        , step(step)
        , chunk_size(chunk_size)
        , count(num_jobs)
        , done(false)
        , task(task)
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

void run_parallel_for(parallel_for_context &ctx) {
    while (true) {
        auto begin = ctx.current.fetch_add(ctx.chunk_size, std::memory_order_relaxed);

        if (begin >= ctx.last) {
            break;
        }

        auto end = std::min(begin + ctx.chunk_size, ctx.last);
        ctx.task(begin, end);
    }
}

void parallel_for_job_func(job::data_type &data) {
    auto archive = memory_input_archive(data.data(), data.size());
    intptr_t ctx_ptr;
    archive(ctx_ptr);
    auto *ctx = reinterpret_cast<parallel_for_context *>(ctx_ptr);

    run_parallel_for(*ctx);

    ctx->decrement();
}

struct parallel_for_async_context {
    using IndexType = size_t;
    std::atomic<IndexType> current;
    const IndexType first;
    const IndexType last;
    const IndexType step;
    const IndexType chunk_size;
    std::atomic<int> completed_counter;
    std::atomic<int> ref_counter;
    task_delegate_t task;
    task_completion_delegate_t completion;

    parallel_for_async_context(IndexType first, IndexType last, IndexType step,
                               IndexType chunk_size, size_t num_jobs,
                               task_delegate_t task, task_completion_delegate_t completion)
        : current(first)
        , first(first)
        , last(last)
        , step(step)
        , chunk_size(chunk_size)
        , completed_counter(0)
        , ref_counter(num_jobs)
        , task(task)
        , completion(completion)
    {}
};

void parallel_for_async_job_func(job::data_type &data) {
    auto archive = memory_input_archive(data.data(), data.size());
    intptr_t ctx_ptr;
    archive(ctx_ptr);
    auto *ctx = reinterpret_cast<parallel_for_async_context *>(ctx_ptr);
    const auto total = ctx->last - ctx->first;

    while (true) {
        auto begin = ctx->current.fetch_add(ctx->chunk_size, std::memory_order_relaxed);
        auto end = std::min(begin + ctx->chunk_size, ctx->last);

        if (begin >= end) {
            break;
        }

        ctx->task(begin, end);

        const auto progress = end - begin;
        const auto completed = ctx->completed_counter.fetch_add(progress, std::memory_order_relaxed) + progress;
        EDYN_ASSERT(completed <= total);

        if (completed == total) {
            if (ctx->completion) {
                ctx->completion();
            }
            break;
        }
    }

    auto ref_count = ctx->ref_counter.fetch_sub(1, std::memory_order_relaxed) - 1;
    EDYN_ASSERT(ref_count >= 0);

    if (ref_count == 0) {
        delete ctx;
    }
}

void enqueue_task_default(task_delegate_t task, unsigned size, task_completion_delegate_t completion) {
    auto &dispatcher = job_dispatcher::global();
    auto num_workers = dispatcher.num_workers();

    // Size of chunk that will be processed per job iteration.
    unsigned count_per_worker_ceil = size / num_workers + (size % num_workers != 0);
    auto chunk_size = std::max(count_per_worker_ceil, 1u);

    // Number of jobs that will be dispatched. Must not be greater than number
    // of workers.
    auto num_jobs = std::min(num_workers, size_t{size});

    // Context that's shared among all jobs. It is deallocated when the last
    // job finishes.
    auto *context = new parallel_for_async_context(0, size, 1, chunk_size, num_jobs, task, completion);

    // Job that'll process chunks of data in worker threads.
    auto child_job = job();
    child_job.func = &parallel_for_async_job_func;
    auto archive = fixed_memory_output_archive(child_job.data.data(), child_job.data.size());
    auto ctx_ptr = reinterpret_cast<intptr_t>(context);
    archive(ctx_ptr);

    // Dispatch background jobs and return immediately after.
    for (size_t i = 0; i < num_jobs; ++i) {
        dispatcher.async(child_job);
    }
}

void enqueue_task_wait_default(task_delegate_t task, unsigned size) {
    auto &dispatcher = job_dispatcher::global();
    unsigned num_workers = dispatcher.num_workers();
    unsigned chunk_size = std::max(size / (num_workers + 1), 1u);
    unsigned num_jobs = std::min(num_workers, size - 1u);
    auto context = parallel_for_context{0u, size, 1u, chunk_size, num_jobs, task};

    auto child_job = job();
    child_job.func = &parallel_for_job_func;
    auto archive = fixed_memory_output_archive(child_job.data.data(), child_job.data.size());
    auto ctx_ptr = reinterpret_cast<intptr_t>(&context);
    archive(ctx_ptr);

    // Dispatch background jobs.
    for (size_t i = 0; i < num_jobs; ++i) {
        dispatcher.async(child_job);
    }

    // Process chunks of the for loop in the current thread as well.
    run_parallel_for(context);

    // Wait all background jobs to finish.
    context.wait();
}

}
