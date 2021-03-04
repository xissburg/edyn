#ifndef EDYN_UTIL_VECTOR_HPP
#define EDYN_UTIL_VECTOR_HPP

#include <vector>
#include <algorithm>

namespace edyn {

template<typename T>
bool vector_contains(const std::vector<T>& vec, const T& val) {
    return std::find(vec.begin(), vec.end(), val) != vec.end();
}

}

#endif // EDYN_UTIL_VECTOR_HPP
