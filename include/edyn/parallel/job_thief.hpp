#ifndef EDYN_PARALLEL_JOB_THIEF_HPP
#define EDYN_PARALLEL_JOB_THIEF_HPP

#include "edyn/parallel/job_queue.hpp"

namespace edyn {

class job_thief {
public:
    void add_queue(job_queue *queue) {
        m_queues.push_back(queue);
    }

    std::shared_ptr<job> try_steal() {
        for (auto *queue : m_queues) {
            auto j = queue->try_pop();
            if (j) {
                return j;
            }
        }
        return {};
    }

private:
    std::vector<job_queue *> m_queues;
};

}

#endif // EDYN_PARALLEL_JOB_THIEF_HPP