#include "edyn/parallel/job_dispatcher.hpp"
#include "edyn/parallel/job_queue_scheduler.hpp"
#include "edyn/config/config.h"
#include <cstdint>

namespace edyn {

thread_local job_queue job_dispatcher::m_queue;

job_dispatcher &job_dispatcher::global() {
    static job_dispatcher instance;
    return instance;
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

void job_dispatcher::async(const job &j) {
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

void job_dispatcher::async(std::thread::id id, const job &j) {
    auto lock = std::shared_lock(m_queues_mutex);
    EDYN_ASSERT(m_queues_map.count(id));
    m_queues_map[id]->push(j);
}

job_queue_scheduler job_dispatcher::get_current_scheduler() {
    auto id = std::this_thread::get_id();
    auto lock = std::shared_lock(m_queues_mutex);
    EDYN_ASSERT(m_queues_map.count(id));
    return job_queue_scheduler(m_queues_map[id]);
}

void job_dispatcher::assure_current_queue() {
    auto id = std::this_thread::get_id();
    // Must not be called from a worker thread.
    EDYN_ASSERT(!m_workers.count(id));

    auto lock = std::lock_guard(m_queues_mutex);
    m_queues_map[id] = &m_queue;
}

void job_dispatcher::once_current_queue() {
    job j;
    while (m_queue.try_pop(j)) {
        j();
    }
}

size_t job_dispatcher::num_workers() const {
    return m_workers.size();
}

}
