#include "edyn/init.hpp"
#include "edyn/parallel/job_dispatcher.hpp"
#include "edyn/comp/island.hpp"
#include "edyn/collision/contact_point.hpp"
#include "edyn/collision/contact_manifold.hpp"
#include "edyn/comp/constraint_row.hpp"
#include "edyn/comp/constraint.hpp"
#include "edyn/comp/island.hpp"
#include "edyn/comp/gravity.hpp"
#include <entt/entt.hpp>

namespace edyn {

static bool g_edyn_initialized {false};

void init() {
    if (g_edyn_initialized) return;

    edyn::job_dispatcher::global().start();

    g_edyn_initialized = true;
}

}