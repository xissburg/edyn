#ifndef EDYN_PARALLEL_ISLAND_WORKER_CONTEXT_HPP
#define EDYN_PARALLEL_ISLAND_WORKER_CONTEXT_HPP

#include <memory>
#include <entt/entity/fwd.hpp>
#include <entt/signal/fwd.hpp>
#include <entt/entity/sparse_set.hpp>
#include "edyn/util/entity_map.hpp"
#include "edyn/parallel/message_queue.hpp"
#include "edyn/parallel/message.hpp"
#include "edyn/parallel/island_worker.hpp"

namespace edyn {

class registry_operation_collection;
class registry_operation_builder;

/**
 * Context of an island worker in the main thread in an island coordinator.
 */
class island_worker_context {

    entt::entity m_island_entity;
    island_worker *m_worker;
    message_queue_in_out m_message_queue;
    bool m_pending_flush;

public:
    entity_map m_entity_map;
    std::unique_ptr<registry_operation_builder> m_op_builder;

    using island_reg_op_func_t = void(entt::entity, msg::island_reg_ops &);
    entt::sigh<island_reg_op_func_t> m_island_reg_op_signal;

    using split_island_func_t = void(entt::entity, msg::split_island &);
    entt::sigh<split_island_func_t> m_split_island_signal;

    island_worker_context(entt::entity island_entity,
                island_worker *worker,
                std::unique_ptr<registry_operation_builder> op_builder,
                message_queue_in_out message_queue);
    ~island_worker_context();

    /**
     * Returns whether there are any pending registry operations to be sent.
     */
    bool reg_ops_empty() const;

    /**
     * Reads messages sent by worker.
     */
    void read_messages();

    /**
     * Sends current registry operations and clears it up, making it ready for more
     * updates.
     */
    void send_reg_ops();

    /**
     * Ensures messages are delivered and processed by waking up the worker
     * in case it is sleeping.
     */
    void flush();

    auto split() {
        return m_worker->split();
    }

    template<typename Message, typename... Args>
    void send(Args &&... args) {
        m_message_queue.send<Message>(std::forward<Args>(args)...);
        m_pending_flush = true;
    }

    void on_island_reg_op(msg::island_reg_ops &);

    auto reg_op_sink() {
        return entt::sink {m_island_reg_op_signal};
    }

    void on_split_island(msg::split_island &);

    auto split_island_sink() {
        return entt::sink {m_split_island_signal};
    }

    auto island_entity() const {
        return m_island_entity;
    }

    /**
     * Schedules worker to be terminated.
     */
    void terminate();
};

}

#endif // EDYN_PARALLEL_ISLAND_WORKER_CONTEXT_HPP
