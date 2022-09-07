#include "edyn/simulation/simulation_worker_context.hpp"
#include "edyn/simulation/simulation_worker.hpp"
#include "edyn/parallel/message.hpp"
#include "edyn/replication/registry_operation_builder.hpp"

namespace edyn {

simulation_worker_context::simulation_worker_context(simulation_worker *worker)
    : m_worker(worker)
    , m_pending_flush(false)
{
}

void simulation_worker_context::flush() {
    if (m_pending_flush) {
        m_worker->reschedule();
        m_pending_flush = false;
    }
}

void simulation_worker_context::terminate() {
    m_worker->terminate();
}

}
