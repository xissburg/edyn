#ifndef EDYN_PARALLEL_ISLAND_WORKER_HPP
#define EDYN_PARALLEL_ISLAND_WORKER_HPP

#include <entt/entt.hpp>
#include "edyn/parallel/buffer_sync.hpp"
#include "edyn/serialization/memory_archive.hpp"
#include "edyn/parallel/message_queue.hpp"

namespace edyn {

class job_dispatcher;

template<typename... Component>
class island_worker_context {
public:
    island_worker_context(buffer_sync_writer writer, 
                          message_consumer consumer, 
                          message_producer producer)
        : m_continuous_loader(m_registry)
        , m_buffer_sync_writer(writer)
        , m_message_consumer(consumer)
        , m_message_producer(producer)
    {
        consumer.sink<registry_snapshot>().connect<&island_worker_context::on_registry_snapshot>(*this);
    }

    void on_registry_snapshot(const registry_snapshot &snapshot) {
        auto input = memory_input_archive(snapshot.data, snapshot.size);
        m_loader
            .entities(input)
            .destroyed(input)
            .component<Component...>(input);
    }

    void sync() {
        auto buffer = memory_output_archive::buffer_type();
        auto output = memory_output_archive(buffer);
        m_registry.snapshot()
            .entities(output)
            .destroyed(output)
            .component<Component...>(output);
        m_buffer_sync_writer.write(buffer);
    }

    friend class island_worker;

private:
    entt::registry m_registry;
    entt::continuous_loader m_loader;
    entt::entity m_island_entity;
    buffer_sync_writer m_buffer_sync_writer;
    message_consumer m_message_consumer;
    message_producer m_message_producer;
    double fixed_dt;
};

class island_worker {
public:
    void operator()();

private:
    island_worker_context *m_ctx;
};

void island_worker_func(job::data_type &);

}

#endif // EDYN_PARALLEL_ISLAND_WORKER_HPP