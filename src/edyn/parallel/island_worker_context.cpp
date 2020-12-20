#include "edyn/parallel/island_worker_context.hpp"
#include "edyn/parallel/island_worker.hpp"

namespace edyn {

island_worker_context::island_worker_context(entt::entity island_entity,
            island_worker *worker,
            message_queue_in_out message_queue)
    : m_island_entity(island_entity)
    , m_worker(worker)
    , m_message_queue(message_queue)
    , m_delta_builder(m_entity_map)
    , m_pending_flush(false)
{
    m_message_queue.sink<registry_delta>().connect<&island_worker_context::on_registry_delta>(*this);
}

island_worker_context::~island_worker_context() {
    m_message_queue.sink<registry_delta>().disconnect(*this);
}

void island_worker_context::on_registry_delta(const registry_delta &delta) {
    m_registry_delta_signal.publish(m_island_entity, delta);
}

bool island_worker_context::delta_empty() const {
    return m_delta_builder.empty();
}

void island_worker_context::read_messages() {
    m_message_queue.update();
}

void island_worker_context::send_delta() {
    send<registry_delta>(std::move(m_delta_builder.get_delta()));
    m_delta_builder.clear();
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