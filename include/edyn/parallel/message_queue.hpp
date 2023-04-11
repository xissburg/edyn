#ifndef EDYN_PARALLEL_MESSAGE_QUEUE_HPP
#define EDYN_PARALLEL_MESSAGE_QUEUE_HPP

#include <mutex>
#include <string>
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

}

#endif // EDYN_PARALLEL_MESSAGE_QUEUE_HPP
