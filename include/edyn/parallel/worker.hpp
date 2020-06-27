#ifndef EDYN_PARALLEL_WORKER_HPP
#define EDYN_PARALLEL_WORKER_HPP

#include <atomic>
#include <memory>
#include "edyn/parallel/job_queue.hpp"
#include "edyn/parallel/job_thief.hpp"

namespace edyn {

class worker {
public:
    void push_job(std::shared_ptr<job> j) {
        m_queue.push(j);
    }

    void run() {
        m_running = true;

        for (;;) {
            for (;;) {
                once();

                auto did_steal = false;

                if (m_thief) {
                    auto j = m_thief->try_steal();
                    if (j) {
                        j->operator()();
                        did_steal = true;
                    }
                }

                if (!did_steal) {
                    break;
                }
            }

            auto j = m_queue.pop();

            if (j) {
                j->operator()();
            }

            if (!m_running) {
                break;
            }
        }
    }

    void once() {
        while (auto j = m_queue.try_pop()) {
            j->operator()();
        }
    }

    void stop() {
        m_running = false;
        m_queue.unblock();
    }

    size_t size() {
        return m_queue.size();
    }

    void set_thief(job_thief *thief) {
        m_thief = thief;
    }

    job_queue &get_queue() {
        return m_queue;
    }

private:
    std::atomic_bool m_running {false};
    job_queue m_queue;
    job_thief *m_thief { nullptr };
};

}

#endif // EDYN_PARALLEL_WORKER_HPP