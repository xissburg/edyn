#ifndef EDYN_SIMULATION_SIMULATION_WORKER_CONTEXT_HPP
#define EDYN_SIMULATION_SIMULATION_WORKER_CONTEXT_HPP

#include <memory>
#include <entt/entity/fwd.hpp>
#include <entt/signal/fwd.hpp>
#include <entt/entity/sparse_set.hpp>
#include "edyn/comp/island.hpp"
#include "edyn/replication/entity_map.hpp"
#include "edyn/parallel/message.hpp"
#include "edyn/simulation/simulation_worker.hpp"
#include "edyn/parallel/message_dispatcher.hpp"

namespace edyn {

/**
 * Context of a simulation worker in the main thread in a stepper async.
 */
class simulation_worker_context {

    simulation_worker *m_worker;
    bool m_pending_flush;

public:
    entity_map m_entity_map;
    double m_timestamp;

    simulation_worker_context(simulation_worker *worker);

    /**
     * Reads messages sent by worker.
     */
    void read_messages();

    /**
     * Ensures messages are delivered and processed by waking up the worker
     * in case it is sleeping.
     */
    void flush();

    template<typename Message, typename... Args>
    void send(message_queue_identifier source, Args &&... args) {
        message_dispatcher::global().send<Message>({"worker"}, source, std::forward<Args>(args)...);
        m_pending_flush = true;
    }

    /**
     * Schedules worker to be terminated.
     */
    void terminate();
};

}

#endif // EDYN_SIMULATION_SIMULATION_WORKER_CONTEXT_HPP
