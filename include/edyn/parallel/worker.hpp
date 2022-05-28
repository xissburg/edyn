#ifndef EDYN_PARALLEL_WORKER_HPP
#define EDYN_PARALLEL_WORKER_HPP

#include <atomic>
#include <memory>
#include "edyn/parallel/job_queue.hpp"

namespace edyn {

/**
 * A worker that runs jobs in a thread.
 */
class worker {
public:
    void push_job(const job &j) {
        m_queue.push(j);
        ++m_size;
    }

    void run() {
        m_running = true;

        for (;;) {
            auto j = m_queue.pop();
            j();
            --m_size;

            if (!m_running) {
                break;
            }
        }
    }

    void once() {
        job j;
        while (m_queue.try_pop(j)) {
            j();
            --m_size;
        }
    }

    void stop() {
        m_running = false;
        m_queue.push(job::noop()); // Unblock queue with a no-op job.
    }

    size_t size() {
        return m_size.load();
    }

    job_queue &get_queue() {
        return m_queue;
    }

private:
    std::atomic_bool m_running {false};
    job_queue m_queue;
    std::atomic<size_t> m_size {0};
};

}

#endif // EDYN_PARALLEL_WORKER_HPP
