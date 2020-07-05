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

    void async(std::shared_ptr<job> j);
    void async(std::thread::id, std::shared_ptr<job> j);

    void assure_current_worker();
    void once_current_worker();

    size_t num_workers() const;

private:
    std::vector<std::unique_ptr<std::thread>> m_threads;
    std::map<std::thread::id, std::unique_ptr<worker>> m_workers;
    job_thief m_thief;

    // Workers for external threads.
    std::map<std::thread::id, std::unique_ptr<worker>> m_other_workers;
    std::mutex m_other_workers_mutex;
};

}

#endif // EDYN_PARALLEL_JOB_DISPATCHER_HPP