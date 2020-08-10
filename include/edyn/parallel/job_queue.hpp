#ifndef EDYN_PARALLEL_JOB_QUEUE_HPP
#define EDYN_PARALLEL_JOB_QUEUE_HPP

#include <mutex>
#include <atomic>
#include <deque>
#include <memory>
#include <condition_variable>
#include "edyn/parallel/job.hpp"

namespace edyn {

class job_queue {
public:
    void push(const job &);

    job pop();

    bool try_pop(job &);

    size_t size();

private:
    std::mutex m_mutex;
    std::deque<job> m_jobs;
    std::condition_variable m_cv;
};

}

#endif // EDYN_PARALLEL_JOB_QUEUE_HPP