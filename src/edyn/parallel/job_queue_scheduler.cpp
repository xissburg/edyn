#include "edyn/parallel/job_queue_scheduler.hpp"
#include "edyn/parallel/job_queue.hpp"

namespace edyn {

void job_queue_scheduler::push(const job &j) {
    m_queue->push(j);
}

}