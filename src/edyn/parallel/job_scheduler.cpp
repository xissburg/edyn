#include "edyn/parallel/job_scheduler.hpp"
#include "edyn/parallel/job_dispatcher.hpp"
#include "edyn/time/time.hpp"
#include "edyn/config/config.h"
#include <algorithm>

namespace edyn {

job_scheduler::job_scheduler(job_dispatcher &dispatcher)
    : m_dispatcher(&dispatcher)
{}

job_scheduler::~job_scheduler() {
    stop();
}

void job_scheduler::start() {
    m_running.store(true, std::memory_order_release);
    m_thread = std::make_unique<std::thread>(&job_scheduler::update, this);
}

void job_scheduler::stop() {
    if (!m_thread) return;

    m_running.store(false, std::memory_order_release);
    m_cv.notify_one();
    m_thread->join();
    m_thread.reset();
}

void job_scheduler::schedule_after(const job &j, double delta_time) {
    EDYN_ASSERT(delta_time > 0);

    auto lock = std::unique_lock(m_mutex);
    auto current_time = (double)performance_counter() / (double)performance_frequency();
    auto job_timestamp = current_time + delta_time;

    auto found_it = std::find_if(m_jobs.begin(), m_jobs.end(), [job_timestamp] (const timed_job &j) {
        return j.m_timestamp > job_timestamp;
    });

    auto did_replace_first = found_it == m_jobs.begin();
    m_jobs.insert(found_it, timed_job{j, job_timestamp});
    lock.unlock();

    if (did_replace_first) {
        // If this new job is going to be inserted at the beginning, it is 
        // necessary to wake up the timer thread so the condition variable
        // can be readjusted to unblock at the earliest time again.
        m_cv.notify_one();
    }
}

void job_scheduler::update() {
    while (m_running.load(std::memory_order_acquire)) {
        auto lock = std::unique_lock(m_mutex);
        
        if (m_jobs.empty()) {
            m_cv.wait(lock, [&] () { return !m_jobs.empty() || !m_running.load(std::memory_order_acquire); });
        }

        auto current_time = (double)performance_counter() / (double)performance_frequency();
        auto it = m_jobs.begin();

        for (; it != m_jobs.end(); ++it) {
            if (it->m_timestamp > current_time) break;
            m_dispatcher->async(it->m_job);
        }

        auto time_until_next_job = it->m_timestamp - current_time;
        m_jobs.erase(m_jobs.begin(), it);

        auto duration = std::chrono::duration<double>(time_until_next_job);
        m_cv.wait_for(lock, duration, [&] () { return !m_running.load(std::memory_order_acquire); });
    }
}

}
