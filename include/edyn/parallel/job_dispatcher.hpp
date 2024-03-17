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
     * Number of background workers.
     */
    size_t num_workers() const;

private:
    std::vector<std::unique_ptr<std::thread>> m_threads;
    std::map<std::thread::id, std::unique_ptr<worker>> m_workers;
    std::atomic<size_t> m_start;
};

}

#endif // EDYN_PARALLEL_JOB_DISPATCHER_HPP
