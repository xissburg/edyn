#include "edyn/parallel/island_worker_context.hpp"
#include "edyn/parallel/island_worker.hpp"
#include "edyn/util/registry_operation_builder.hpp"

namespace edyn {

island_worker_context::island_worker_context(entt::entity island_entity,
            island_worker *worker,
            std::unique_ptr<registry_operation_builder> op_builder,
            message_queue_in_out message_queue)
    : m_island_entity(island_entity)
    , m_worker(worker)
    , m_message_queue(message_queue)
    , m_op_builder(std::move(op_builder))
    , m_pending_flush(false)
{
    m_message_queue.sink<msg::island_reg_ops>().connect<&island_worker_context::on_island_reg_op>(*this);
    m_message_queue.sink<msg::split_island>().connect<&island_worker_context::on_split_island>(*this);
}

island_worker_context::~island_worker_context() {
    m_message_queue.sink<msg::island_reg_ops>().disconnect(*this);
    m_message_queue.sink<msg::split_island>().disconnect(*this);
}

void island_worker_context::on_island_reg_op(msg::island_reg_ops &msg) {
    m_island_reg_op_signal.publish(m_island_entity, msg);
}

void island_worker_context::on_split_island(msg::split_island &msg) {
    m_split_island_signal.publish(m_island_entity, msg);
}

bool island_worker_context::reg_ops_empty() const {
    return m_op_builder->empty();
}

void island_worker_context::read_messages() {
    m_message_queue.update();
}

void island_worker_context::send_reg_ops() {
    send<msg::island_reg_ops>(m_op_builder->finish());
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
