#ifndef EDYN_PARALLEL_PARALLEL_FOR_ASYNC_HPP
#define EDYN_PARALLEL_PARALLEL_FOR_ASYNC_HPP

#include "edyn/parallel/parallel_for.hpp"

namespace edyn {

namespace detail {

template<typename IndexType, typename Function>
class for_loop_range_pool_async {
public:
    using for_loop_range_type = for_loop_range<IndexType, Function, atomic_counter>;

    /**
     * @brief Constructs the pool with one initial root for-loop containing
     * the whole range.
     */
    for_loop_range_pool_async(IndexType first, IndexType last, 
                              IndexType step, job_dispatcher &dispatcher,
                              job &j, Function func)
        : m_first(first)
        , m_last(last)
        , m_step(step)
        , m_func(func)
        , m_loop_counter(j, 1, dispatcher)
        , m_ref_count(1)
    {
        m_root = new for_loop_range_type(first, last, step, func, m_loop_counter);
        m_head.store(m_root, std::memory_order_relaxed);
    }

    ~for_loop_range_pool_async() {
        // Destroy linked list of for-loops.
        auto *loop = m_head.load(std::memory_order_relaxed);
        
        while (loop) {
            auto next = loop->m_next;
            delete loop;
            loop = next;
        }
    }

    for_loop_range_type *create(IndexType first, IndexType last) {
        // Insert new loop into linked list in a thread-safe and lock-free 
        // manner using `compare_exchange_weak`.
        auto *loop = new for_loop_range_type(first, last, m_step, m_func, m_loop_counter);
        loop->m_next = m_head.load(std::memory_order_relaxed);

        while (!m_head.compare_exchange_weak(loop->m_next, loop,
                                             std::memory_order_release,
                                             std::memory_order_relaxed));

        return loop;
    }

    for_loop_range_type *root() {
        return m_root;
    }

    // Called before stealing a range of another loop preceding a call to `create`.
    void will_create() {
        m_loop_counter.increment();
    }

    void add_ref() {
        m_ref_count.fetch_add(1, std::memory_order_relaxed);
    }

    int release() {
        auto ref_count = m_ref_count.fetch_add(-1, std::memory_order_relaxed) - 1;
        EDYN_ASSERT(ref_count >= 0);
        return ref_count;
    }

private:
    const IndexType m_first;
    const IndexType m_last;
    const IndexType m_step;
    Function m_func;
    atomic_counter m_loop_counter;
    std::atomic<for_loop_range_type *> m_head;
    for_loop_range_type *m_root;
    std::atomic<int> m_ref_count;
};

template<typename IndexType, typename Function>
struct parallel_for_async_context {
    using for_loop_range_pool_type = for_loop_range_pool_async<IndexType, Function>;
    using for_loop_range_type = for_loop_range<IndexType, Function, atomic_counter>;

    for_loop_range_pool_type *m_loop_pool;
    job_dispatcher *m_dispatcher;
    for_loop_range_type *m_parent;

    parallel_for_async_context()
        : m_loop_pool(nullptr)
        , m_dispatcher(nullptr)
        , m_parent(nullptr)
    {}

    parallel_for_async_context(for_loop_range_pool_type *loop_pool,
                               job_dispatcher *dispatcher,
                               for_loop_range_type *parent)
        : m_loop_pool(loop_pool)
        , m_dispatcher(dispatcher)
        , m_parent(parent)
    {}
};

template<typename Archive, typename IndexType, typename Function>
void serialize(Archive &archive, parallel_for_async_context<IndexType, Function> &ctx) {
    if constexpr(Archive::is_output::value) {
        auto intptr = reinterpret_cast<intptr_t>(ctx.m_loop_pool);
        archive(intptr);
        intptr = reinterpret_cast<intptr_t>(ctx.m_dispatcher);
        archive(intptr);
        intptr = reinterpret_cast<intptr_t>(ctx.m_parent);
        archive(intptr);
    } else {
        intptr_t intptr;
        archive(intptr);
        ctx.m_loop_pool = reinterpret_cast<for_loop_range_pool_async<IndexType, Function> *>(intptr);
        archive(intptr);
        ctx.m_dispatcher = reinterpret_cast<job_dispatcher *>(intptr);
        archive(intptr);
        ctx.m_parent = reinterpret_cast<for_loop_range<IndexType, Function, atomic_counter> *>(intptr);
    }
}

template<typename IndexType, typename Function>
void parallel_for_async_partition(job::data_type &data) {
    auto archive = memory_input_archive(data.data(), data.size());
    auto ctx = parallel_for_async_context<IndexType, Function>();
    archive(ctx);

    auto pool = ctx.m_loop_pool;
    auto dispatcher = ctx.m_dispatcher;
    auto parent = ctx.m_parent;

    // Decrement job count and if zero delete shared loop pool on exit.
    auto defer = std::shared_ptr<void>(nullptr, [pool] (void *) { 
        auto ref_count = pool->release();
        if (ref_count == 0) delete pool;
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

    // Return null if the parent loop is nearly done (i.e. the remaining work
    // is under a percentage of the total).
    if (remaining * 100 < total * 10) {
        return;
    }

    // Prepare to steal a range from `parent`.
    // Increment loop counter in advance. It is important to increment it before
    // range stealing below, since it might terminate the parent right after and
    // cause the loop counter to be decremented to zero thus causing a wait on it
    // to return prematurely resulting in an incomplete run of the entire for-loop
    // range.
    pool->will_create();

    // Effectively steal a range of work from candidate by moving its last
    // index to the halfway point between current and last.
    auto middle = current + remaining / 2;
    parent->m_last.store(middle, std::memory_order_release);

    // It is possible that by the time `middle` is stored in `parent->m_last`,
    // `parent->m_current` is greater than `middle` since the for-loop
    // is running while this range stealing is taking place. If that is the case,
    // the parent loop would be terminated at this point since middle is assigned
    // to last, and the loop exits when current >= last. To prevent calling
    // `m_func` more than once for the elements between `middle` and 
    // `parent->m_current`, load `parent->m_current` and check if it's greater
    // than or equals to `middle` and if so, start from there instead.
    current = parent->m_current.load(std::memory_order_acquire);
    auto new_first = current >= middle ? current : middle;

    auto *loop = pool->create(new_first, last);

    // Dispatch one child job that will split the parent job range if it is
    // executed before the parent job is nearly done.
    {
        pool->add_ref();

        auto child_job = job();
        child_job.func = &parallel_for_async_partition<IndexType, Function>;

        auto archive = fixed_memory_output_archive(child_job.data.data(), child_job.data.size());
        auto ctx = parallel_for_async_context<IndexType, Function>(pool, dispatcher, parent);
        archive(ctx);

        dispatcher->async(child_job);
    }

    // Dispatch another child job which will split this range if it is executed
    // before this job is nearly done.
    {
        pool->add_ref();

        auto child_job = job();
        child_job.func = &parallel_for_async_partition<IndexType, Function>;

        auto archive = fixed_memory_output_archive(child_job.data.data(), child_job.data.size());
        auto ctx = parallel_for_async_context<IndexType, Function>(pool, dispatcher, loop);
        archive(ctx);

        dispatcher->async(child_job);
    }

    loop->run();
}

template<typename IndexType, typename Function>
void parallel_for_async_job_func(job::data_type &data) {
    auto archive = memory_input_archive(data.data(), data.size());
    auto ctx = parallel_for_async_context<IndexType, Function>();
    archive(ctx);

    auto pool = ctx.m_loop_pool;
    auto dispatcher = ctx.m_dispatcher;

    // Decrement job count and if zero delete shared loop pool on exit.
    auto defer = std::shared_ptr<void>(nullptr, [pool] (void *) { 
        auto ref_count = pool->release();
        if (ref_count == 0) delete pool;
    });
    
    // Create child job which will steal a range of the for-loop if it gets a
    // chance to be executed before the loop is finished.
    {
        pool->add_ref();

        auto child_job = job();
        child_job.func = &detail::parallel_for_async_partition<IndexType, Function>;

        auto archive = fixed_memory_output_archive(child_job.data.data(), child_job.data.size());
        auto ctx = detail::parallel_for_async_context<IndexType, Function>(pool, dispatcher, pool->root());
        archive(ctx);

        dispatcher->async(child_job);
    }

    pool->root()->run();
}

} // namespace detail

template<typename IndexType, typename Function>
void parallel_for_async(job_dispatcher &dispatcher, IndexType first, IndexType last, IndexType step, job &j, Function func) {
    EDYN_ASSERT(step > IndexType{0});
    EDYN_ASSERT(first < last);
    EDYN_ASSERT(last - first > IndexType{1});

    // The last job to finish running will delete `pool`.
    auto *pool = new detail::for_loop_range_pool_async<IndexType, Function>(first, last, step, dispatcher, j, func);

    // Dispatch root for-loop job.
    auto root_job = job();
    root_job.func = &detail::parallel_for_async_job_func<IndexType, Function>;

    auto archive = fixed_memory_output_archive(root_job.data.data(), root_job.data.size());
    auto ctx = detail::parallel_for_async_context<IndexType, Function>(pool, &dispatcher, pool->root());
    archive(ctx);

    dispatcher.async(root_job);
}

}

#endif // EDYN_PARALLEL_PARALLEL_FOR_ASYNC_HPP
