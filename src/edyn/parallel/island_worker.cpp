#include "edyn/parallel/island_worker.hpp"
#include "edyn/parallel/job.hpp"
#include "edyn/comp/island.hpp"
#include "edyn/time/time.hpp"
#include "edyn/parallel/job_dispatcher.hpp"
#include "edyn/serialization/memory_archive.hpp"

namespace edyn {

void island_worker_func(job::data_type &data) {
    auto archive = memory_input_archive(data.data(), data.size());
    intptr_t ctx_intptr;
    archive(ctx_intptr);
    auto *ctx = reinterpret_cast<island_worker_context_base *>(ctx_intptr);

    if (ctx->is_finished()) {
        // `ctx` is dynamically allocated and it reschedules itself after every
        // invocation to `update`. If it is marked as finished, it should be
        // deallocated thus stopping the rescheduling cycle.
        delete ctx;
    } else {
        ctx->update();
    }
}

}