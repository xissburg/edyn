#ifndef EDYN_PARALLEL_MESSAGE_QUEUE_HPP
#define EDYN_PARALLEL_MESSAGE_QUEUE_HPP

#include <array>
#include <cstdint>
#include <mutex>
#include <shared_mutex>
#include <queue>
#include <cstdint>
#include <algorithm>

#include <entt/core/family.hpp>
#include <entt/signal/sigh.hpp>
#include <entt/core/type_traits.hpp>

namespace edyn {

/**
 * @brief A message queue intended for single-producer single-consumer usage
 * accross different threads.
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
            auto messages = std::vector<Message>(std::move(m_messages));
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
        uint16_t runtime_type;
    };

    template<typename Message>
    static auto type() {
        if constexpr(entt::is_named_type_v<Message>) {
            return entt::named_type_traits<Message>::value;
        } else {
            return message_family::type<Message>;
        }
    }

    template<typename Message>
    signal_wrapper<Message> & assure() {
        auto wtype = type<Message>();
        typed_wrapper *typedw = nullptr;

        if constexpr(entt::is_named_type_v<Message>) {
            const auto it = std::find_if(m_wrappers.begin(), m_wrappers.end(), 
                [wtype] (const auto &candidate) {
                return candidate.wrapper && candidate.runtime_type == wtype;
            });

            if (it == m_wrappers.cend()) {
                auto lock = std::lock_guard(m_mutex);
                typedw = &m_wrappers.emplace_back();
            } else {
                typedw = &(*it);
            }
        } else {
            if (!(wtype < m_wrappers.size())) {
                auto lock = std::lock_guard(m_mutex);
                m_wrappers.resize(wtype + 1);
            }

            typedw = &m_wrappers[wtype];

            if (typedw->wrapper && typedw->runtime_type != wtype) {
                auto lock = std::lock_guard(m_mutex);
                m_wrappers.emplace_back();
                std::swap(m_wrappers[wtype], m_wrappers.back());
                typedw = &m_wrappers[wtype];
            }
        }

        if (!typedw->wrapper) {
            typedw->wrapper = std::make_unique<signal_wrapper<Message>>();
            typedw->runtime_type = wtype;
        }

        return static_cast<signal_wrapper<Message> &>(*typedw->wrapper);
    }

    template<typename Message>
    using sink_type = typename signal_wrapper<Message>::sink_type;

    template<typename Message>
    sink_type<Message> sink() {
        return assure<Message>().sink();
    }

    template<typename Message>
    void push(Message &&msg) {
        // Expected to be called from the producer thread only.
        assure<std::decay_t<Message>>().push(std::forward<Message>(msg));
    }

    void update() const {
        // Expected to be called from the consumer thread only.
        auto lock = std::shared_lock(m_mutex);
        for (auto &w : m_wrappers) {
            w.wrapper->publish();
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

    template<typename T>
    void send(T &&msg) {
        m_queue->push(std::forward<T>(msg));
    }
private:
    std::shared_ptr<message_queue> m_queue;
};

class message_queue_output {
public:
    message_queue_output(std::shared_ptr<message_queue> queue) : m_queue(queue) {}

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

    template<typename T>
    void send(T &&msg) {
        m_input.send(std::forward<T>(msg));
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