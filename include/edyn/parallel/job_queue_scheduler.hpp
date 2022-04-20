#ifndef EDYN_PARALLEL_JOB_QUEUE_SCHEDULER_HPP
#define EDYN_PARALLEL_JOB_QUEUE_SCHEDULER_HPP

#include <cstdint>

namespace edyn {

class job_queue;
struct job;

/**
 * Encapsulates a `job_queue` and only allows pushing jobs into it.
 */
class job_queue_scheduler {
public:
    job_queue_scheduler(job_queue *queue = nullptr)
        : m_queue(queue)
    {}

    void push(const job &);

    template<typename Archive>
    friend void serialize(Archive &, job_queue_scheduler &);

private:
    job_queue *m_queue;
};

template<typename Archive>
void serialize(Archive &archive, job_queue_scheduler& scheduler) {
    if constexpr(Archive::is_input::value) {
        intptr_t intptr;
        archive(intptr);
        scheduler.m_queue = reinterpret_cast<job_queue *>(intptr);
    } else {
        auto intptr = reinterpret_cast<intptr_t>(scheduler.m_queue);
        archive(intptr);
    }
}

}

#endif // EDYN_PARALLEL_JOB_QUEUE_SCHEDULER_HPP
