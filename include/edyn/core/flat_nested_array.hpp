#ifndef EDYN_CORE_FLAT_NESTED_ARRAY_HPP
#define EDYN_CORE_FLAT_NESTED_ARRAY_HPP

#include <cstddef>
#include <vector>
#include "edyn/config/config.h"

namespace edyn {

/**
 * Stores an array of arrays in a single vector along with a list containing
 * the index where each subrange starts.
 */
template<typename T>
class flat_nested_array {
public:
    class inner_array {
    public:
        inner_array(const flat_nested_array<T> *parent, size_t range_start, size_t size)
            : m_parent(parent)
            , m_range_start(range_start)
            , m_size(size)
        {}

        const T & operator[](size_t idx) const {
            EDYN_ASSERT(idx < m_size);
            return m_parent->m_data[m_range_start + idx];
        }

        size_t size() const {
            return m_size;
        }

    private:
        const flat_nested_array<T> *m_parent;
        size_t m_range_start;
        size_t m_size;
    };

    /**
     * Starts a new nested array.
     */
    void push_array() {
        m_range_starts.push_back(m_data.size());
    }

    /**
     * Appends new value into the last nested array.
     */
    void push_back(const T &value) {
        m_data.push_back(value);
    }

    /**
     * Number of nested arrays.
     */
    size_t size() const {
        return m_range_starts.size();
    }

    /**
     * Reserves data for the contiguous nested arrays.
     */
    void reserve_data(size_t count) {
        m_data.reserve(count);
    }

    /**
     * Reserves data for the nested subranges, i.e. the number of nested arrays.
     */
    void reserve_nested(size_t count) {
        m_range_starts.reserve(count);
    }

    /**
     * Returns the nested array at the given index.
     */
    inner_array operator[](size_t idx) const {
        EDYN_ASSERT(idx < m_range_starts.size());
        auto next_idx = idx + 1;
        auto range_start = m_range_starts[idx];
        auto range_end = next_idx < m_range_starts.size() ? m_range_starts[next_idx] : m_data.size();
        auto range_size = range_end - range_start;
        return inner_array(this, range_start, range_size);
    }

    template<typename Archive, typename U>
    friend void serialize(Archive &, flat_nested_array<U> &);

    template<typename U>
    friend size_t serialization_sizeof(const flat_nested_array<U> &);

private:
    std::vector<T> m_data;
    std::vector<size_t> m_range_starts;
};

}

#endif // EDYN_CORE_FLAT_NESTED_ARRAY_HPP
