#ifndef EDYN_SERIALIZATION_S11N_UTIL_HPP
#define EDYN_SERIALIZATION_S11N_UTIL_HPP

#include <cstdint>
#include <type_traits>

namespace edyn {

template<typename Archive, typename Enum>
void serialize_enum(Archive &archive, Enum &value) {
    using underlying_type = std::underlying_type_t<Enum>;
    if constexpr(Archive::is_input::value) {
        underlying_type i {};
        archive(i);
        value = Enum{i};
    } else {
        auto i = static_cast<underlying_type>(value);
        archive(i);
    }
}

template<typename Archive, typename T>
void serialize_pointer(Archive &archive, T **ptr) {
    if constexpr(Archive::is_input::value) {
        intptr_t ptr_int {};
        archive(ptr_int);
        *ptr = reinterpret_cast<T *>(ptr_int);
    } else {
        auto ptr_int = reinterpret_cast<intptr_t>(*ptr);
        archive(ptr_int);
    }
}

}

#endif // EDYN_SERIALIZATION_S11N_UTIL_HPP
