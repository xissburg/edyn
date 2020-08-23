#ifndef EDYN_PARALLEL_ISLAND_WORKER_HPP
#define EDYN_PARALLEL_ISLAND_WORKER_HPP

#include <entt/entt.hpp>
#include "edyn/parallel/buffer_sync.hpp"
#include "edyn/serialization/memory_archive.hpp"
#include "edyn/parallel/message_queue.hpp"
#include "edyn/parallel/message.hpp"
#include "edyn/parallel/job.hpp"
#include "edyn/parallel/job_dispatcher.hpp"
#include "edyn/time/time.hpp"
#include "edyn/collision/broadphase.hpp"
#include "edyn/collision/narrowphase.hpp"
#include "edyn/dynamics/solver.hpp"
#include "edyn/comp/island.hpp"
#include "edyn/comp/relation.hpp"

namespace edyn {

class job_dispatcher;

void island_worker_func(job::data_type &);

class island_worker_context_base {
public:
    virtual void update() = 0;
};

template<typename... Component>
class island_worker_context: public island_worker_context_base {
public:
    island_worker_context(buffer_sync_writer writer, 
                          message_queue_in_out message_queue)
        : m_loader(m_registry)
        , m_buffer_sync_writer(writer)
        , m_message_queue(message_queue)
        , m_bphase(m_registry)
        , m_nphase(m_registry)
        , m_sol(m_registry)
    {
        m_message_queue.sink<msg::registry_snapshot>().connect<&island_worker_context::on_registry_snapshot>(*this);
    }

    void on_registry_snapshot(const msg::registry_snapshot &snapshot) {
        auto input = memory_input_archive(snapshot.data.data(), snapshot.data.size());
        m_loader
            .entities(input)
            .destroyed(input)
            .component<Component...>(input);
    }

    void sync() {
        auto buffer = memory_output_archive::buffer_type();
        auto output = memory_output_archive(buffer);
        m_loader.snapshot().component<Component...>(output, &relation::entity);
        m_buffer_sync_writer.write(buffer);
    }

    void update() override {
        // Process messages.
        m_message_queue.update();

        auto &isle = m_registry.get<island>(m_island_entity);
        auto timestamp = (double)performance_counter() / (double)performance_frequency();
        auto dt = timestamp - isle.timestamp;

        if (dt < fixed_dt) {
            auto j = job();
            j.func = &island_worker_func;
            auto archive = fixed_memory_output_archive(j.data.data(), j.data.size());
            auto ctx_intptr = reinterpret_cast<intptr_t>(this);
            archive(ctx_intptr);

            //job_dispatcher::global()::async_after(isle.timestamp + fixed_dt - timestamp, j);
            job_dispatcher::global().async(j);
            
            return;
        }

        m_bphase.update();
        m_nphase.update();
        m_sol.update(0, dt);

        isle.timestamp += dt;

        sync();
    }

private:
    entt::registry m_registry;
    entt::continuous_loader m_loader;
    entt::entity m_island_entity;
    broadphase m_bphase;
    narrowphase m_nphase;
    solver m_sol;
    buffer_sync_writer m_buffer_sync_writer;
    message_queue_in_out m_message_queue;
    double fixed_dt;
};

}

#endif // EDYN_PARALLEL_ISLAND_WORKER_HPP