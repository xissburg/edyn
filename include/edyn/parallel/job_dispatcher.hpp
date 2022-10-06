#ifndef EDYN_PARALLEL_JOB_DISPATCHER_HPP
#define EDYN_PARALLEL_JOB_DISPATCHER_HPP

#include <map>
#include <atomic>
#include <vector>
#include <thread>
#include <mutex>
#include <shared_mutex>
#include "edyn/parallel/worker.hpp"

namespace edyn {

struct job;
class job_queue;
class job_queue_scheduler;

/**
 * Manages a set of worker threads and dispatches jobs to them.
 */
class job_dispatcher {
public:
    static job_dispatcher &global();

    ~job_dispatcher();

    void start(size_t num_worker_threads);

    void stop();

    bool running() const;

    /**
     * Schedules a job to run asynchronously in a worker thread.
     */
    void async(const job &);

    /**
     * Schedules a job to run in a specific thread.
     */
    void async(std::thread::id, const job &);

    /**
     * Schedules a job to run in a specific queue.
     */
    void async(size_t queue_index, const job &);

    /**
     * Get the index of the queue for the current thread.
     */
    size_t current_queue_index() const;

    /**
     * Instantiates a `job_queue` for the current thread internally if it hasn't
     * already. Must be called before jobs are scheduled in that thread.
     */
    void assure_current_queue();

    /**
     * Executes all pending jobs in the current thread's queue.
     */
    void once_current_queue();

    /**
     * Gets a scheduler for the current thread which can be shared with other
     * threads for message passing.
     */
    job_queue_scheduler get_current_scheduler();

    /**
     * Number of background workers.
     */
    size_t num_workers() const;

private:
    std::vector<std::unique_ptr<std::thread>> m_threads;
    std::map<std::thread::id, std::unique_ptr<worker>> m_workers;

    // Job queue for regular threads.
    std::vector<job_queue *> m_queues;
    std::map<std::thread::id, size_t> m_queues_map;
    mutable std::shared_mutex m_queues_mutex;

    // Job queue for this thread.
    static thread_local job_queue m_queue;

    std::atomic<size_t> m_start;
};

}

#endif // EDYN_PARALLEL_JOB_DISPATCHER_HPP
