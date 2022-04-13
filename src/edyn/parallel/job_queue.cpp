#include "edyn/parallel/job_queue.hpp"
#include "edyn/config/config.h"

namespace edyn {

void job_queue::push(const job &j) {
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_jobs.push(j);
    }
    m_cv.notify_one();
}

job job_queue::pop() {
    std::unique_lock<std::mutex> lock(m_mutex);
    m_cv.wait(lock, [&]() {
        return !m_jobs.empty();
    });

    auto j = m_jobs.front();
    m_jobs.pop();
    return j;
}

bool job_queue::try_pop(job &j) {
    std::lock_guard<std::mutex> lock(m_mutex);
    if (m_jobs.empty()) {
        return false;
    }

    j = m_jobs.front();
    m_jobs.pop();
    return true;
}

size_t job_queue::size() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_jobs.size();
}

}
