#include "edyn/parallel/job_dispatcher.hpp"
#include "edyn/config/config.h"

namespace edyn {

job_dispatcher &job_dispatcher::global() {
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
    EDYN_ASSERT(m_workers.empty());

    for (size_t i = 0; i < num_worker_threads; ++i) {
        auto w = std::make_unique<worker>();
        m_thief.add_queue(&w->get_queue());
        w->set_thief(&m_thief);

        auto t = std::make_unique<std::thread>(&worker::run, w.get());
        auto id = t->get_id();

        m_threads.push_back(std::move(t));
        m_workers[id] = std::move(w);
    }
}

void job_dispatcher::stop() {
    for (auto &pair : m_workers) {
        pair.second->stop();
    }

    for (auto &t : m_threads) {
        t->join();
    }

    m_workers.clear();
    m_threads.clear();
}

void job_dispatcher::async(std::shared_ptr<job> j) {
    EDYN_ASSERT(!m_workers.empty());

    auto best_id = std::thread::id();
    auto min_num_jobs = SIZE_MAX;
    for (auto &pair : m_workers) {
        auto s = pair.second->size();
        if (s < min_num_jobs) {
            min_num_jobs = s;
            best_id = pair.first;
        }
    }

    EDYN_ASSERT(m_workers.count(best_id));

    m_workers[best_id]->push_job(j);
}

void job_dispatcher::async(std::thread::id id, std::shared_ptr<job> j) {
    std::lock_guard<std::mutex> lock(m_external_workers_mutex);
    EDYN_ASSERT(m_external_workers.count(id));

    // Must not be called from a worker thread.
    EDYN_ASSERT(!m_workers.count(id));
    
    m_external_workers[id]->push_job(j);
}

void job_dispatcher::assure_current_worker() {
    std::lock_guard<std::mutex> lock(m_external_workers_mutex);
    auto id = std::this_thread::get_id();

    // Must not be called from a worker thread.
    EDYN_ASSERT(!m_workers.count(id));

    if (m_external_workers.count(id)) {
        return;
    }

    m_external_workers[id] = std::make_unique<worker>();
}

void job_dispatcher::once_current_worker() {
    std::lock_guard<std::mutex> lock(m_external_workers_mutex);
    auto id = std::this_thread::get_id();

    // Must not be called from a worker thread.
    EDYN_ASSERT(!m_workers.count(id));

    EDYN_ASSERT(m_external_workers.count(id));
    m_external_workers[id]->once();
}

size_t job_dispatcher::num_workers() const {
    return m_workers.size();
}

}
