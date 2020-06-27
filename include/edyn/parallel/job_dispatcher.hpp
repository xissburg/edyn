#ifndef EDYN_PARALLEL_JOB_DISPATCHER_HPP
#define EDYN_PARALLEL_JOB_DISPATCHER_HPP

#include <thread>
#include <memory>
#include "edyn/parallel/worker.hpp"

namespace edyn {

struct worker_thread {
    std::unique_ptr<worker> w;
    std::unique_ptr<std::thread> t;
};

class job_dispatcher {
public:
    static job_dispatcher &shared();

    ~job_dispatcher();

    void start(size_t num_worker_threads = 8);

    void stop();

    void async(std::shared_ptr<job> j);

    size_t num_workers() const;

private:
    std::vector<worker_thread> m_worker_threads;
    job_thief m_thief;
};

}

#endif // EDYN_PARALLEL_JOB_DISPATCHER_HPP