#include "edyn/parallel/island_worker_context.hpp"
#include "edyn/parallel/island_worker.hpp"
#include "edyn/parallel/message.hpp"
#include "edyn/util/registry_operation_builder.hpp"

namespace edyn {

island_worker_context::island_worker_context(island_worker *worker,
                                             std::unique_ptr<registry_operation_builder> op_builder)
    : m_worker(worker)
    , m_op_builder(std::move(op_builder))
    , m_pending_flush(false)
{
}

bool island_worker_context::reg_ops_empty() const {
    return m_op_builder->empty();
}

void island_worker_context::send_reg_ops(message_queue_identifier source) {
    send<msg::update_entities>(source, m_op_builder->finish());
}

void island_worker_context::flush() {
    if (m_pending_flush) {
        m_worker->reschedule();
        m_pending_flush = false;
    }
}

void island_worker_context::terminate() {
    m_worker->terminate();
}

}
