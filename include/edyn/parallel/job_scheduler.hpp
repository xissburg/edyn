#ifndef EDYN_PARALLEL_JOB_SCHEDULER_HPP
#define EDYN_PARALLEL_JOB_SCHEDULER_HPP

#include <mutex>
#include <thread>
#include <memory>
#include <vector>
#include <atomic>
#include <condition_variable>
#include "edyn/parallel/job.hpp"

namespace edyn {

class job_dispatcher;

/**
 * Schedules jobs for execution at a later time in a `job_dispatcher`.
 */
class job_scheduler final {
    struct timed_job {
        job m_job;
        double m_timestamp;
    };

    void update();

public:
    job_scheduler(job_dispatcher &);
    ~job_scheduler();

    void start();
    void stop();

    void schedule_after(const job &, double delta_time);

private:
    job_dispatcher *m_dispatcher;
    std::unique_ptr<std::thread> m_thread;
    std::vector<timed_job> m_jobs;
    std::mutex m_mutex;
    std::condition_variable m_cv;
    std::atomic_bool m_running;
};

}

#endif // EDYN_PARALLEL_JOB_SCHEDULER_HPP
