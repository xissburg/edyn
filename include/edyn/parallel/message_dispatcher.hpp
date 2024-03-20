#ifndef EDYN_PARALLEL_MESSAGE_DISPATCHER_HPP
#define EDYN_PARALLEL_MESSAGE_DISPATCHER_HPP

#include <mutex>
#include <shared_mutex>
#include <unordered_map>
#include <utility>
#include <vector>
#include "edyn/config/config.h"
#include "edyn/parallel/message_queue.hpp"

namespace edyn {

template<typename... MessageTypes>
class message_queue_handle {

    template<typename T>
    void maybe_consume_message(any_message &msg) {
        if (entt::type_id<T>() == msg.content.type()) {
            using signal_type = entt::sigh<void(message<T> &)>;
            auto m = message<T>{msg.sender, std::move(*entt::any_cast<T>(&msg.content))};
            std::get<signal_type>(m_signals).publish(m);
        }
    }

public:
    const message_queue_identifier identifier;

    message_queue_handle(message_queue_identifier identifier, message_queue &queue)
        : identifier(identifier)
        , m_queue(&queue)
    {}

    template<typename MessageType>
    auto sink() {
        using signal_type = entt::sigh<void(message<MessageType> &)>;
        return entt::sink{std::get<signal_type>(m_signals)};
    }

    void update() {
        m_queue->consume([&] (any_message &msg) {
            (maybe_consume_message<MessageTypes>(msg), ...);
        });
    }

    auto push_sink() {
        return m_queue->push_sink();
    }

private:
    std::tuple<entt::sigh<void(message<MessageTypes> &)> ...> m_signals;
    message_queue *m_queue;
};

class message_dispatcher {
public:
    static message_dispatcher &global();

    template<typename... MessageTypes>
    auto make_queue(const std::string &name) {
        auto lock = std::lock_guard(m_queues_mutex);
        EDYN_ASSERT(!m_queues.count(name));
        m_queues[name] = std::make_unique<message_queue>();
        return message_queue_handle<MessageTypes...>({name}, *m_queues.at(name));
    }

    template<typename T, typename... Args>
    void send(message_queue_identifier destination, message_queue_identifier source, Args&& ... args) {
        auto lock = std::shared_lock(m_queues_mutex);
        if (m_queues.count(destination.value)) {
            m_queues.at(destination.value)->push<T>(source, std::forward<Args>(args)...);
        }
    }

    void clear_queues() {
        auto lock = std::shared_lock(m_queues_mutex);
        m_queues.clear();
    }

private:
    std::unordered_map<std::string, std::unique_ptr<message_queue>> m_queues;
    mutable std::shared_mutex m_queues_mutex;
};

}

#endif // EDYN_PARALLEL_MESSAGE_DISPATCHER_HPP
