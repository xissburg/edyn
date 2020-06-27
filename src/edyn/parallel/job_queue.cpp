#include "edyn/parallel/job_queue.hpp"
#include "edyn/config/config.h"

namespace edyn {

void job::operator()() {
    std::lock_guard<std::mutex> lock(m_mutex);
    EDYN_ASSERT(!m_done);
    run();
    m_done = true;
    m_cv.notify_one();
}

void job::join() {
    std::unique_lock<std::mutex> lock(m_mutex);
    m_cv.wait(lock, [&] () {
        return m_done;
    });
}

void job_queue::push(std::shared_ptr<job> j) {
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_jobs.push_back(j);
    }
    m_cv.notify_one();
}

std::shared_ptr<job> job_queue::pop() {
    std::unique_lock<std::mutex> lock(m_mutex);
    m_cv.wait(lock, [&] () {
        return !m_jobs.empty() || m_unblock;
    });

    if (m_unblock) {
        m_unblock = false;
        return {};
    }

    auto j = m_jobs.back();
    m_jobs.pop_back();
    return j;
}

std::shared_ptr<job> job_queue::try_pop() {
    std::lock_guard<std::mutex> lock(m_mutex);
    if (m_jobs.empty()) {
        return {};
    }

    auto j = m_jobs.back();
    m_jobs.pop_back();
    return j;
}

void job_queue::unblock() {
    m_unblock = true;
    m_cv.notify_one();
}

size_t job_queue::size() {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_jobs.size();
}

}