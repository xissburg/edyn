#include "edyn/parallel/simulation_worker_context.hpp"
#include "edyn/parallel/simulation_worker.hpp"
#include "edyn/parallel/message.hpp"
#include "edyn/util/registry_operation_builder.hpp"

namespace edyn {

simulation_worker_context::simulation_worker_context(simulation_worker *worker,
                                             std::unique_ptr<registry_operation_builder> op_builder)
    : m_worker(worker)
    , m_op_builder(std::move(op_builder))
    , m_pending_flush(false)
{
}

bool simulation_worker_context::reg_ops_empty() const {
    return m_op_builder->empty();
}

void simulation_worker_context::send_reg_ops(message_queue_identifier source) {
    send<msg::update_entities>(source, m_op_builder->finish());
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
