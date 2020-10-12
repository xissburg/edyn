#include <type_traits>
#include "edyn/dynamics/world.hpp"
#include "edyn/sys/update_presentation.hpp"
#include "edyn/time/time.hpp"
#include "edyn/comp.hpp"
#include "edyn/collision/contact_manifold.hpp"
#include "edyn/parallel/job_dispatcher.hpp"
#include "edyn/serialization/s11n.hpp"
#include "edyn/dynamics/island_util.hpp"
#include "edyn/parallel/registry_snapshot.hpp"

namespace edyn {

world::world(entt::registry &reg) 
    : m_registry(&reg)
    , m_solver(reg)
    , m_bphase(reg)
    , m_island_coordinator(reg)
{
    job_dispatcher::global().assure_current_queue();
}

world::~world() {
    
}

void world::update() {
    // Run jobs scheduled in physics thread.
    job_dispatcher::global().once_current_queue();

    m_island_coordinator.update();
    m_bphase.update();

    if (m_paused) {
        snap_presentation(*m_registry);
    } else {
        auto time = (double)performance_counter() / (double)performance_frequency();
        update_presentation(*m_registry, time);
    }
}

void world::set_paused(bool paused) {
    m_paused = paused;
    m_island_coordinator.set_paused(paused);
}

void world::step() {
    m_island_coordinator.step_simulation();
}

}