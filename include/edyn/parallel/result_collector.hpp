#ifndef EDYN_PARALLEL_RESULT_COLLECTOR_HPP
#define EDYN_PARALLEL_RESULT_COLLECTOR_HPP

#include <array>
#include <atomic>
#include "edyn/config/config.h"

namespace edyn {

/**
 * @brief Thread-safe utility to store results of a parallel computation.
 */
template<typename T, size_t N>
class result_collector {
    struct block {
        std::array<T, N> array;
        block *next {nullptr};
        block *prev {nullptr};
        size_t index {0};
    };

public:
    result_collector()
        : m_last_block(&m_first_block)
        , m_block_count(1)
        , m_size(0)
    {}

    ~result_collector() {
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

        if (index == capacity) {
            auto *last_block = new block;
            last_block->prev = m_last_block.load(std::memory_order_relaxed);
            last_block->prev->next = last_block;
            last_block->index = last_block->prev->index + 1;
            m_last_block.store(last_block, std::memory_order_release);
            m_block_count.fetch_add(1, std::memory_order_release);
        } else if (index > capacity) {
            // Spin until the capacity grows.
            while (1) {
                auto curr_count = m_block_count.load(std::memory_order_acquire);
                if (curr_count != block_count) {
                    // This value could be destined to be inserted in a block after
                    // the one that was just added, hmm...
                    auto curr_capacity = curr_count * N;
                    if (index >= curr_capacity) {
                        EDYN_ASSERT(false);
                    }
                    break;
                }
            }
        }

        auto index_in_block = index % N;
        auto block_index = index / N;
        auto *block = m_last_block.load(std::memory_order_acquire);
        while (block->index != block_index) {
            block = block->prev;
        }
        block->array[index_in_block] = value;
    }

    template<typename Function>
    void each(Function func) {
        auto *block = &m_first_block;
        auto size = m_size.load(std::memory_order_relaxed);
        
        for (size_t i = 0; i < size; ++i) {
            auto block_index = i % N;

            if (block_index == 0 && i > 0) {
                block = block->next;
            }

            func(block->array[block_index]);
        }
    }

private:
    block m_first_block;
    std::atomic<block *> m_last_block;
    std::atomic<size_t> m_block_count;
    std::atomic<size_t> m_size;
};

}

#endif // EDYN_PARALLEL_RESULT_COLLECTOR_HPP
