#include "edyn/parallel/job_dispatcher.hpp"
#include "edyn/config/config.h"

namespace edyn {

job_dispatcher &job_dispatcher::shared() {
    static job_dispatcher singleton;
    return singleton;
}

job_dispatcher::~job_dispatcher() {
    stop();
}

void job_dispatcher::start() {
    auto num_threads = std::thread::hardware_concurrency();

    if (num_threads == 0) {
        num_threads = 8;
    }

    start(num_threads);
}

void job_dispatcher::start(size_t num_worker_threads) {
    EDYN_ASSERT(m_worker_threads.empty());
    m_worker_threads.resize(num_worker_threads);

    for (auto &wt : m_worker_threads) {
        wt.w = std::make_unique<worker>();
        m_thief.add_queue(&wt.w->get_queue());
        wt.w->set_thief(&m_thief);
    }

    for (auto &wt : m_worker_threads) {
        wt.t = std::make_unique<std::thread>(&worker::run, wt.w.get());
    }
}

void job_dispatcher::stop() {
    for (auto &wt : m_worker_threads) {
        wt.w->stop();
    }

    for (auto &wt : m_worker_threads) {
        wt.t->join();
    }

    m_worker_threads.clear();
}

void job_dispatcher::async(std::shared_ptr<job> j) {
    EDYN_ASSERT(!m_worker_threads.empty());

    auto best_idx = SIZE_MAX;
    auto min_num_jobs = SIZE_MAX;
    for (size_t i = 0; i < m_worker_threads.size(); ++i) {
        auto s = m_worker_threads[i].w->size();
        if (s < min_num_jobs) {
            min_num_jobs = s;
            best_idx = i;
        }
    }

    m_worker_threads[best_idx].w->push_job(j);
}

size_t job_dispatcher::num_workers() const {
    return m_worker_threads.size();
}

}
