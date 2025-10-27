#ifndef EDYN_CORE_FREE_LIST_HPP
#define EDYN_CORE_FREE_LIST_HPP

#include "edyn/config/config.h"
#include <utility>
#include <vector>

namespace edyn {

template <typename T, typename IndexType, IndexType null_index>
class free_list {
    static constexpr size_t allocation_size = 16;

public:
    IndexType insert(T &&value) {
        if (m_free_list == null_index) {
            m_free_list = m_elem.size();
            m_elem.resize(m_elem.size() + allocation_size);

            for (auto i = m_free_list; i < m_elem.size(); ++i) {
                auto &elem = m_elem[i];
                elem = {};
                elem.next = i + 1;
            }

            m_elem.back().next = null_index;
        }

        auto index = m_free_list;
        EDYN_ASSERT(index < m_elem.size());
        auto &elem = m_elem[index];
        m_free_list = elem.next;
        elem = std::move(value);
        ++m_size;

        return index;
    }

    void remove(IndexType index) {
        EDYN_ASSERT(index < m_elem.size());
        m_elem[index] = {};
        m_elem[index].next = m_free_list;
        m_free_list = index;
        --m_size;
    }

    void clear() {
        for (auto i = size_t{}; i < m_elem.size(); ++i) {
            auto &elem = m_elem[i];
            elem.next = i + 1;
        }

        if (m_elem.empty()) {
            m_free_list = null_index;
        } else {
            m_elem.back().next = null_index;
            m_free_list = 0;
        }

        m_size = 0;
    }

    const T & operator[](IndexType index) const {
        EDYN_ASSERT(index < m_elem.size());
        return m_elem[index];
    }

    T & operator[](IndexType index) {
        return const_cast<T &>(std::as_const(*this)[index]);
    }

    // Total number of valid elements.
    auto count() const {
        return m_size;
    }

    auto empty() const {
        return m_size == 0;
    }

    // Valid indices go from 0 up to `range() -1`.
    auto range() const {
        return m_elem.size();
    }

private:
    std::vector<T> m_elem;
    size_t m_size {};
    IndexType m_free_list {null_index};
};

}

#endif // EDYN_CORE_FREE_LIST_HPP
