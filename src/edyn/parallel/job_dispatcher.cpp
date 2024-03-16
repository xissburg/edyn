#include "edyn/parallel/job_dispatcher.hpp"
#include "edyn/parallel/job_queue.hpp"
#include "edyn/parallel/worker.hpp"
#include "edyn/config/config.h"
#include <cstdint>

namespace edyn {

job_dispatcher &job_dispatcher::global() {
    static job_dispatcher instance;
    return instance;
}

job_dispatcher::~job_dispatcher() {
    stop();
}

void job_dispatcher::start(size_t num_worker_threads) {
    EDYN_ASSERT(num_worker_threads > 0);
    EDYN_ASSERT(m_workers.empty());

    for (size_t i = 0; i < num_worker_threads; ++i) {
        auto w = std::make_unique<worker>();
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

bool job_dispatcher::running() const {
    return !m_threads.empty();
}

void job_dispatcher::async(const job &j) {
    EDYN_ASSERT(!m_workers.empty());

    if (m_workers.size() == 1) {
        m_workers.begin()->second->push_job(j);
        return;
    }

    // Find least busy worker to insert job into. Start search from a different
    // worker each time to create a better spread. This prevents the first
    // worker from being prioritized and getting most jobs.
    auto best_id = std::thread::id();
    auto min_num_jobs = SIZE_MAX;
    auto start = m_start.fetch_add(1, std::memory_order_relaxed) % m_workers.size();
    auto end = start + m_workers.size();

    for (size_t i = start; i < end; ++i) {
        auto k = i % m_workers.size();
        auto first = m_workers.begin();
        std::advance(first, k);
        auto &pair = *first;

        auto s = pair.second->size();
        if (s == 0) {
            pair.second->push_job(j);
            return;
        }
        if (s < min_num_jobs) {
            min_num_jobs = s;
            best_id = pair.first;
        }
    }

    EDYN_ASSERT(m_workers.count(best_id));

    m_workers[best_id]->push_job(j);
}

size_t job_dispatcher::num_workers() const {
    return m_workers.size();
}

}
