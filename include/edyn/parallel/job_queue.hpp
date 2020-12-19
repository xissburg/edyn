#ifndef EDYN_PARALLEL_JOB_QUEUE_HPP
#define EDYN_PARALLEL_JOB_QUEUE_HPP

#include <mutex>
#include <queue>
#include <condition_variable>
#include "edyn/parallel/job.hpp"

namespace edyn {

/**
 * Thread-safe queue of jobs.
 */
class job_queue {
public:
    void push(const job &);

    job pop();

    bool try_pop(job &);

    size_t size() const;

private:
    mutable std::mutex m_mutex;
    std::queue<job> m_jobs;
    std::condition_variable m_cv;
};

}

#endif // EDYN_PARALLEL_JOB_QUEUE_HPP