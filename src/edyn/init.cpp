#include "edyn/init.hpp"
#include "edyn/parallel/job_dispatcher.hpp"

namespace edyn {

static bool g_edyn_initialized {false};

void init() {
    if (g_edyn_initialized) return;

    edyn::job_dispatcher::global().start();

    g_edyn_initialized = true;
}

}