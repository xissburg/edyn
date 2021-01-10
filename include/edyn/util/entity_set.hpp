#ifndef EDYN_UTIL_ENTITY_COLLECTION_HPP
#define EDYN_UTIL_ENTITY_COLLECTION_HPP

#include <vector>
#include <entt/fwd.hpp>
#include <initializer_list>

namespace edyn {

class entity_set final {
public:
    class iterator final {
        friend class entity_set;
        
        iterator(const entity_set &set, size_t index)
            : m_set(&set), m_index(index)
        {}

    public:
        iterator & operator++() {
            ++m_index;
            return *this;
        }

        iterator operator++(int) {
            auto orig = *this;
            ++m_index;
            return orig;
        }

        iterator & operator--() {
            --m_index;
            return *this;
        }

        iterator operator--(int) {
            auto orig = *this;
            --m_index;
            return orig;
        }

        iterator & operator+=(size_t i) {
            m_index += i;
            return *this;
        }

        iterator & operator+(size_t i) const {
            auto copy = *this;
            return (copy += i);
        }

        iterator & operator-=(size_t i) {
            m_index -= i;
            return *this;
        }

        iterator & operator-(size_t i) const {
            auto copy = *this;
            return (copy -= i);
        }

        bool operator==(const iterator &other) const {
            return other.m_index == m_index;
        }

        bool operator!=(const iterator &other) const {
            return !(*this == other);
        }

        entt::entity operator*() const {
            return m_set->m_data[m_index];
        }

    private:
        const entity_set *m_set;
        size_t m_index;
    };

    entity_set() {}

    entity_set(std::initializer_list<entt::entity> list)
        : m_data(list)
    {}

    template<typename It>
    entity_set(It first, It last)
        : m_data(first, last)
    {}

    void insert(entt::entity entity) {
        for (auto e : m_data) {
            if (e == entity) return;
        }
        m_data.push_back(entity);
    }

    template<typename It>
    void insert(It first, It last) {
        for (auto it = first; it != last; ++it) {
            insert(*it);
        }
    }

    void erase(entt::entity entity) {
        for (size_t i = 0; i < m_data.size(); ++i) {
            if (m_data[i] == entity) {
                m_data[i] = m_data.back();
                m_data.pop_back();
                return;
            }
        }
    }

    void clear() {
        m_data.clear();
    }

    void reserve(size_t amount) {
        m_data.reserve(amount);
    }

    bool contains(entt::entity entity) const {
        for (auto e : m_data) {
            if (e == entity) return true;
        }
        return false;
    }

    size_t size() const {
        return m_data.size();
    }

    bool empty() const {
        return m_data.empty();
    }

    iterator begin() const {
        return iterator(*this, 0);
    }

    iterator end() const {
        return iterator(*this, m_data.size());
    }

    friend class iterator;

private:
    std::vector<entt::entity> m_data;
};

}

#endif // EDYN_UTIL_ENTITY_COLLECTION_HPP