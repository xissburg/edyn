#ifndef EDYN_PARALLEL_RESULT_ACCUMULATOR_HPP
#define EDYN_PARALLEL_RESULT_ACCUMULATOR_HPP

#include <array>
#include <atomic>
#include "edyn/config/config.h"

namespace edyn {

template<typename T, size_t N>
class result_accumulator {
    struct block {
        std::array<T, N> array;
        block *next {nullptr};
    };

public:
    result_accumulator()
        : m_last_block(&m_first_block)
        , m_block_count(1)
        , m_size(0)
        , m_lock(false)
    {}

    ~result_accumulator() {
        auto *block = m_first_block.next;

        while (block) {
            auto *next = block->next;
            delete block;
            block = next;
        }
    }

    void append(const T& value) {
        auto index = m_size.fetch_add(1, std::memory_order_relaxed);
        auto block_count = m_block_count.load(std::memory_order_relaxed);
        auto capacity = block_count * N;
        auto block_index = index % N;

        if (index == capacity) {
            auto *last_block = new block;
            m_last_block->next = last_block;
            m_last_block = last_block;
            m_block_count.fetch_add(1, std::memory_order_release);
        } else if (index > capacity) {
            while (1) {
                auto curr_count = m_block_count.load(std::memory_order_acquire);
                if (curr_count != block_count) {
                    auto curr_capacity = curr_count * N;
                    EDYN_ASSERT(index < curr_capacity);
                    break;
                }
            }
        }

        m_last_block->array[block_index] = value;
    }

    template<typename Function>
    void each(Function func) {
        auto *block = &m_first_block;
        auto size = m_size.load(std::memory_order_relaxed);
        
        for (size_t i = 0; i < size; ++i) {
            auto block_index = i % N;

            if (i > 0 && block_index == 0) {
                block = block->next;
            }

            func(block->array[block_index]);
        }
    }

private:
    block m_first_block;
    block *m_last_block;
    std::atomic<size_t> m_block_count;
    std::atomic<size_t> m_size;
    std::atomic<bool> m_lock;
};

}

#endif // EDYN_PARALLEL_RESULT_ACCUMULATOR_HPP
