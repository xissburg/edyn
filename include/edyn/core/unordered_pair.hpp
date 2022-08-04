#ifndef EDYN_CORE_UNORDERED_PAIR_HPP
#define EDYN_CORE_UNORDERED_PAIR_HPP

#include "edyn/config/config.h"

namespace edyn {

/**
 * A pair of values of the same type which tests equals independent of order.
 */
template<typename T>
struct unordered_pair {
    T first;
    T second;

    unordered_pair() = default;
    unordered_pair(T first, T second)
        : first(first)
        , second(second)
    {}

    auto operator[](unsigned idx) const {
        EDYN_ASSERT(idx < 2);
        return idx == 0 ? first : second;
    }

    bool operator==(const unordered_pair &other) const {
        return (first == other.first && second == other.second) ||
               (first == other.second && second == other.first);
    }

    bool operator<(const unordered_pair &other) const {
        if (first == other.second && second == other.first) {
            return false;
        }
        if (first == other.first) {
            return second < other.second;
        }
        return first < other.first;
    }
};

}

#endif // EDYN_CORE_UNORDERED_PAIR_HPP
