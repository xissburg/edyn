#ifndef EDYN_PARALLEL_JOB_QUEUE_HPP
#define EDYN_PARALLEL_JOB_QUEUE_HPP

#include <mutex>
#include <atomic>
#include <vector>
#include <memory>
#include <condition_variable>
#include "edyn/parallel/job.hpp"

namespace edyn {

class job_queue {
public:
    void push(std::shared_ptr<job> j);

    std::shared_ptr<job> pop();

    std::shared_ptr<job> try_pop();

    void unblock();

    size_t size();

private:
    std::mutex m_mutex;
    std::vector<std::shared_ptr<job>> m_jobs;
    std::condition_variable m_cv;
    std::atomic_bool m_unblock {false};
};

}

#endif // EDYN_PARALLEL_JOB_QUEUE_HPP