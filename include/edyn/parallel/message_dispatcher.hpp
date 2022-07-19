#ifndef EDYN_PARALLEL_MESSAGE_DISPATCHER_HPP
#define EDYN_PARALLEL_MESSAGE_DISPATCHER_HPP

#include <mutex>
#include <unordered_map>
#include <utility>
#include <vector>
#include <entt/core/any.hpp>
#include <entt/signal/sigh.hpp>

namespace edyn {

struct message_queue_identifier {
    std::string value;
};

struct any_message {
    message_queue_identifier sender;
    entt::any content;
};

template<typename T>
struct message {
    message_queue_identifier sender;
    T content;
};

class message_queue {
public:
    template<typename T, typename... Args>
    void push(message_queue_identifier source, Args&& ... args) {
        // TODO: create a new single-producer/single-consumer queue for each new source.
        auto lock = std::lock_guard(m_mutex);
        m_messages.emplace_back(any_message{source, entt::any(std::in_place_type_t<T>{}, std::forward<Args>(args)...)});
        m_push_signal.publish();
    }

    template<typename Func>
    void consume(Func func) {
        auto lock = std::unique_lock(m_mutex);
        auto messages = std::move(m_messages);
        lock.unlock();

        for (auto &msg : messages) {
            func(msg);
        }
    }

    auto push_sink() {
        return entt::sink{m_push_signal};
    }

private:
    mutable std::mutex m_mutex;
    std::vector<any_message> m_messages;
    entt::sigh<void(void)> m_push_signal;
};

template<typename... MessageTypes>
class message_queue_handle {

    template<typename T>
    void maybe_consume_message(any_message &msg) {
        if (entt::type_id<T>() == msg.content.type()) {
            using signal_type = entt::sigh<void(const message<T> &)>;
            auto m = message<T>{msg.sender, std::move(*entt::any_cast<T>(&msg.content))};
            std::get<signal_type>(m_signals).publish(std::move(m));
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
        using signal_type = entt::sigh<void(const message<MessageType> &)>;
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
    std::tuple<entt::sigh<void(const message<MessageTypes> &)> ...> m_signals;
    message_queue *m_queue;
};

class message_dispatcher {
public:
    static message_dispatcher &global();

    template<typename... MessageTypes>
    auto make_queue(const std::string &name) {
        m_queues[name] = std::make_unique<message_queue>();
        return message_queue_handle<MessageTypes...>({name}, *m_queues.at(name));
    }

    template<typename T, typename... Args>
    void send(message_queue_identifier destination, message_queue_identifier source, Args&& ... args) {
        m_queues.at(destination.value)->push<T>(source, std::forward<Args>(args)...);
    }

private:
    std::unordered_map<std::string, std::unique_ptr<message_queue>> m_queues;
};

}

#endif // EDYN_PARALLEL_MESSAGE_DISPATCHER_HPP
