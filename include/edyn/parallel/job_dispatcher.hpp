#ifndef EDYN_PARALLEL_JOB_DISPATCHER_HPP
#define EDYN_PARALLEL_JOB_DISPATCHER_HPP

#include <thread>
#include "edyn/parallel/worker.hpp"

namespace edyn {

struct worker_thread {
    worker w;
    std::thread t;
};

constexpr size_t g_num_worker_threads = 8;

class job_dispatcher {
public:
    static job_dispatcher &shared() {
        static job_dispatcher singleton;
        return singleton;
    }

    void start() {
        for (auto &wt : m_worker_threads) {
            m_thief.add_queue(&wt.w.get_queue());
            wt.w.set_thief(&m_thief);
        }

        for (auto &wt : m_worker_threads) {
            wt.t = std::thread(&worker::run, &wt.w);
        }
    }

    void async(std::shared_ptr<job> j) {
        auto best_idx = SIZE_MAX;
        auto min_num_jobs = SIZE_MAX;
        for (size_t i = 0; i < m_worker_threads.size(); ++i) {
            auto s = m_worker_threads[i].w.size();
            if (s < min_num_jobs) {
                min_num_jobs = s;
                best_idx = i;
            }
        }

        m_worker_threads[best_idx].w.push_job(j);
    }

private:
    std::array<worker_thread, g_num_worker_threads> m_worker_threads;
    job_thief m_thief;
};

}

#endif // EDYN_PARALLEL_JOB_DISPATCHER_HPP