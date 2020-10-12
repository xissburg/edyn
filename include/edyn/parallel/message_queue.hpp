#ifndef EDYN_PARALLEL_MESSAGE_QUEUE_HPP
#define EDYN_PARALLEL_MESSAGE_QUEUE_HPP

#include <array>
#include <cstdint>
#include <mutex>
#include <shared_mutex>
#include <queue>
#include <cstdint>
#include <algorithm>

#include <entt/entt.hpp>

namespace edyn {

/**
 * @brief A message queue intended for single-producer single-consumer usage
 * between two threads.
 */
class message_queue {
    // Based on `entt::dispatcher`.
    using message_family = entt::family<struct internal_message_family>;

    struct base_wrapper {
        virtual ~base_wrapper() = default;
        virtual void publish() = 0;
    };

    template<typename Message>
    struct signal_wrapper: base_wrapper {
        using signal_type = entt::sigh<void(const Message &)>;
        using sink_type = typename signal_type::sink_type;

        template<typename... Args>
        void push(Args &&... args) {
            // Expected to be called from the producer thread only.
            auto lock = std::lock_guard(m_mutex);
            m_messages.emplace_back(std::forward<Args>(args)...);
        }

        void publish() override {
            // Expected to be called from the consumer thread only.
            auto lock = std::unique_lock(m_mutex);
            auto messages = std::move(m_messages);
            lock.unlock();

            for (auto &msg : messages) {
                m_signal.publish(msg);
            }
        }

        sink_type sink() {
            return entt::sink{m_signal};
        }

    private:
        std::mutex m_mutex;
        signal_type m_signal{};
        std::vector<Message> m_messages;
    };

    struct typed_wrapper {
        std::unique_ptr<base_wrapper> wrapper;
        ENTT_ID_TYPE runtime_type;
    };

    template<typename Message>
    static auto type() {
        return entt::type_info<Message>::id();
    }

    template<typename Message>
    signal_wrapper<Message> & assure() {
        auto wtype = type<Message>();
        typed_wrapper *typedw = nullptr;

        const auto it = std::find_if(m_wrappers.begin(), m_wrappers.end(), 
            [wtype] (const auto &candidate) {
            return candidate.wrapper && candidate.runtime_type == wtype;
        });

        if (it == m_wrappers.cend()) {
            auto lock = std::lock_guard(m_mutex);
            typedw = &m_wrappers.emplace_back();
            typedw->wrapper = std::make_unique<signal_wrapper<Message>>();
            typedw->runtime_type = wtype;
        } else {
            typedw = &(*it);
        }

        return static_cast<signal_wrapper<Message> &>(*typedw->wrapper);
    }

    template<typename Message>
    using sink_type = typename signal_wrapper<Message>::sink_type;

    template<typename Message>
    sink_type<Message> sink() {
        return assure<Message>().sink();
    }

    template<typename Message, typename... Args>
    void push(Args &&... args) {
        // Expected to be called from the producer thread only.
        assure<std::decay_t<Message>>().push(std::forward<Args>(args)...);
    }

    void update() const {
        // Expected to be called from the consumer thread only.
        auto lock = std::shared_lock(m_mutex);
        for (auto &w : m_wrappers) {
            if (w.wrapper) {
                w.wrapper->publish();
            }
        }
    }

    friend class message_queue_input;
    friend class message_queue_output;

    mutable std::shared_mutex m_mutex;
    std::vector<typed_wrapper> m_wrappers;
};

class message_queue_input {
public:
    message_queue_input(std::shared_ptr<message_queue> queue) : m_queue(queue) {}
    message_queue_input(const message_queue_input &) = default;

    template<typename Message, typename... Args>
    void send(Args &&... args) {
        m_queue->push<Message>(std::forward<Args>(args)...);
    }
private:
    std::shared_ptr<message_queue> m_queue;
};

class message_queue_output {
public:
    message_queue_output(std::shared_ptr<message_queue> queue) : m_queue(queue) {}
    message_queue_output(const message_queue_output &) = default;
    
    void update() const {
        m_queue->update();
    }

    template<typename T>
    auto sink() {
        return m_queue->sink<T>();
    }

private:
    std::shared_ptr<message_queue> m_queue;
};

inline
auto make_message_queue_input_output() {
    auto queue = std::make_shared<message_queue>();
    return std::make_pair(message_queue_input(queue), message_queue_output(queue));
}

class message_queue_in_out {
public:
    message_queue_in_out(message_queue_input input, message_queue_output output)
        : m_input(input)
        , m_output(output)
    {}

    message_queue_in_out(const message_queue_in_out &) = default;

    template<typename Message, typename... Args>
    void send(Args &&... args) {
        m_input.send<Message>(std::forward<Args>(args)...);
    }

    void update() const {
        m_output.update();
    }

    template<typename T>
    auto sink() {
        return m_output.sink<T>();
    }

private:
    message_queue_input m_input;
    message_queue_output m_output;
};

}

#endif // EDYN_PARALLEL_MESSAGE_QUEUE_HPP