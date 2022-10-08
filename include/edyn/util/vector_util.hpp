#ifndef EDYN_UTIL_VECTOR_UTIL_HPP
#define EDYN_UTIL_VECTOR_UTIL_HPP

#include <vector>
#include <algorithm>

namespace edyn {

template<typename T>
bool vector_contains(const std::vector<T> &vec, const T &val) {
    return std::find(vec.begin(), vec.end(), val) != vec.end();
}

template<typename T>
void vector_erase(std::vector<T> &vec, const T &val) {
    vec.erase(std::remove(vec.begin(), vec.end(), val), vec.end());
}

}

#endif // EDYN_UTIL_VECTOR_UTIL_HPP
