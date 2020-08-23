#ifndef EDYN_PARALLEL_MESSAGE_QUEUE_HPP
#define EDYN_PARALLEL_MESSAGE_QUEUE_HPP

#include <array>
#include <cstdint>
#include <mutex>
#include <queue>
#include <entt/core/family.hpp>

namespace edyn {

class message_queue {
    using message_family = entt::family<struct internal_message_family>;

    template<typename Message>
    std::queue<Message> & assure() {

    }

    template<typename Message>
    void push(const Message &msg) {
        std::lock_guard lock(m_mutex);
        assure<Message>().push(msg);
    }

    friend class message_producer;
    friend class message_consumer;

    std::mutex m_mutex;
    std::vector<wrapper_data> wrappers;
};

class message_producer {
public:
    template<typename T>
    void send(const T &msg) {
        m_queue->push(msg);
    }
private:
    std::shared_ptr<message_queue> m_queue;
}

class message_consumer {
public:
    void update() {
        
    }

    template<typename T>
    sink_type sink() {

    }

private:
    std::shared_ptr<message_queue> m_queue;
}

}

#endif // EDYN_PARALLEL_MESSAGE_QUEUE_HPP