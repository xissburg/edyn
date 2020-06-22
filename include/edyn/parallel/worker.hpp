#ifndef EDYN_PARALLEL_WORKER_HPP
#define EDYN_PARALLEL_WORKER_HPP

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
            j->operator()();
        }
    }

    void once() {
        while (auto j = m_queue.try_pop()) {
            j->operator()();
        }
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
    job_queue m_queue;
    job_thief *m_thief { nullptr };
};

}

#endif // EDYN_PARALLEL_WORKER_HPP