#ifndef EDYN_UTIL_ARRAY_UTIL_HPP
#define EDYN_UTIL_ARRAY_UTIL_HPP

#include <array>

namespace edyn {

// Create array filled with given value. From https://stackoverflow.com/a/41259045
namespace detail {
    template <typename T, std::size_t...Is>
    constexpr std::array<T, sizeof...(Is)> make_array(const T& value, std::index_sequence<Is...>) {
        return {{(static_cast<void>(Is), value)...}};
    }
}

template <std::size_t N, typename T>
constexpr std::array<T, N> make_array(const T& value) {
    return detail::make_array(value, std::make_index_sequence<N>());
}

}

#endif // EDYN_UTIL_ARRAY_UTIL_HPP
