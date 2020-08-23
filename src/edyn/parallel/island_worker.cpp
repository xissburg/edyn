#include "edyn/parallel/island_worker.hpp"
#include "edyn/parallel/job.hpp"
#include "edyn/comp/island.hpp"
#include "edyn/time/time.hpp"
#include "edyn/parallel/job_dispatcher.hpp"
#include "edyn/serialization/memory_archive.hpp"

namespace edyn {

void island_worker::operator()() {
    // Process messages.
    m_ctx->m_message_consumer.update();

    auto &isle = m_ctx->m_registry.get<island>(m_ctx->m_island_entity);
    auto timestamp = (double)performance_counter() / (double)performance_frequency();
    auto dt = timestamp - isle.timestamp;

    if (dt < fixed_dt) {
        auto j = job();
        j.func = &island_worker_func;
        auto archive = fixed_memory_output_archive(j.data.data(), j.data.size());
        auto ctx_intptr = reinterpret_cast<intptr_t>(this);
        archive(ctx_intptr);

        job_dispatcher::global()::async_after(isle.timestamp + fixed_dt - timestamp, j);
        
        return;
    }

    bphase.update();
    nphase.update();
    sol.update(step_, dt);

    isle.timestamp += dt;

    m_ctx->sync();
}

void island_worker_func(job::data_type &data) {
    auto archive = memory_input_archive(data.data(), data.size());
    intptr_t ctx_intptr;
    archive(ctx_intptr);
    auto *ctx = reinterpret_cast<island_worker_context *>(ctx_intptr);

    (*ctx)();
}

}