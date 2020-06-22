#ifndef EDYN_PARALLEL_WORKER_HPP
#define EDYN_PARALLEL_WORKER_HPP

#include "edyn/parallel/job_queue.hpp"

namespace edyn {

class worker {
public:
    void append(job &j) {
        m_queue.push(j);
    }

    void run() {
        for (;;) {
            auto j = m_queue.pop();
            j();
        }
    }

    void once() {
        job j([](){});
        while (m_queue.maybe_pop(j)) {
            j();
        }
    }

    size_t size() {
        return m_queue.size();
    }

private:
    job_queue m_queue;
};

}

#endif // EDYN_PARALLEL_WORKER_HPP