#ifndef EDYN_PARALLEL_JOB_DISPATCHER_HPP
#define EDYN_PARALLEL_JOB_DISPATCHER_HPP

#include <map>
#include <vector>
#include <memory>
#include <thread>
#include <mutex>
#include "edyn/parallel/worker.hpp"

namespace edyn {

class job_dispatcher {
public:
    static job_dispatcher &global();

    ~job_dispatcher();

    void start();
    void start(size_t num_worker_threads);

    void stop();

    /**
     * Schedules a job to run asynchronously in a worker thread.
     */
    void async(std::shared_ptr<job> j);

    /**
     * Schedules a job to run in a worker in a specific thread.
     */
    void async(std::thread::id, std::shared_ptr<job> j);

    /**
     * Instantiates a worker for the current thread internally if it hasn't
     * been instantiated yet. Must be called before jobs are scheduled in that
     * thread.
     */
    void assure_current_worker();

    /**
     * Runs the current worker once, thus executing all pending jobs in the
     * current thread.
     */
    void once_current_worker();

    /**
     * Number of background workers.
     */
    size_t num_workers() const;

private:
    std::vector<std::unique_ptr<std::thread>> m_threads;
    std::map<std::thread::id, std::unique_ptr<worker>> m_workers;
    job_thief m_thief;

    // Workers for external threads.
    std::map<std::thread::id, std::unique_ptr<worker>> m_external_workers;
    std::mutex m_external_workers_mutex;
};

}

#endif // EDYN_PARALLEL_JOB_DISPATCHER_HPP