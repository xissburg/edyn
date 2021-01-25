#ifndef EDYN_PARALLEL_ISLAND_WORKER_CONTEXT_HPP
#define EDYN_PARALLEL_ISLAND_WORKER_CONTEXT_HPP

#include <memory>
#include <entt/fwd.hpp>
#include "edyn/util/entity_set.hpp"
#include "edyn/util/entity_map.hpp"
#include "edyn/parallel/message_queue.hpp"
#include "edyn/parallel/island_delta_builder.hpp"

namespace edyn {

class island_worker;
/**
 * Context of an island worker in the main thread in an island coordinator.
 */
class island_worker_context {

    entt::entity m_island_entity;
    island_worker *m_worker;
    message_queue_in_out m_message_queue;
    bool m_pending_flush;

public:
    entity_set m_entities;
    entity_map m_entity_map;
    std::unique_ptr<island_delta_builder> m_delta_builder;

    using island_delta_func_t = void(entt::entity, const island_delta &);
    entt::sigh<island_delta_func_t> m_island_delta_signal;

    island_worker_context(entt::entity island_entity,
                island_worker *worker,
                message_queue_in_out message_queue);
    ~island_worker_context();

    /**
     * Returns whether the current delta doesn't contain any changes.
     */
    bool delta_empty() const;

    /**
     * Reads messages sent by worker.
     */
    void read_messages();

    /**
     * Sends current registry delta and clears it up, making it ready for more
     * updates.
     */
    void send_delta();

    /**
     * Ensures messages are delivered and processed by waking up the worker
     * in case it is sleeping.
     */
    void flush();

    template<typename Message, typename... Args>
    void send(Args &&... args) {
        m_message_queue.send<Message>(std::forward<Args>(args)...);
        m_pending_flush = true;
    }
    
    void on_island_delta(const island_delta &);

    auto island_delta_sink() {
        return entt::sink {m_island_delta_signal};
    }

    /**
     * Schedules worker to be terminated.
     */
    void terminate();
};

}

#endif // EDYN_PARALLEL_ISLAND_WORKER_CONTEXT_HPP