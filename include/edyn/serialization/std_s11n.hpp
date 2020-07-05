#ifndef EDYN_SERIALIZATION_STD_S11N_HPP
#define EDYN_SERIALIZATION_STD_S11N_HPP

#include <vector>
#include <cstdint>
#include <memory>
#include <type_traits>

namespace edyn {

template<typename Archive, typename T>
void serialize(Archive &archive, std::vector<T> &vector) {
    auto size = vector.size();
    archive(size);
    vector.resize(size);

    for (size_t i = 0; i < size; ++i) {
        archive(vector[i]);
    }
}

template<typename Archive>
void serialize(Archive &archive, std::vector<bool> &vector) {
    auto size = vector.size();
    archive(size);
    vector.resize(size);

    // Serialize individual bits.
    using set_type = uint32_t;
    constexpr auto set_num_bits = sizeof(set_type) * 8;
    // Number of sets of bits of size `set_num_bits`.
    // Use ceiling on integer division.
    const auto num_sets = size / set_num_bits + (size % set_num_bits != 0); 

    for (size_t i = 0; i < num_sets; ++i) {
        const auto start = i * set_num_bits;
        const auto count = std::min(size - start, set_num_bits);

        if constexpr(Archive::is_output::value) {
            set_type set = 0;
            for (size_t j = 0; j < count; ++j) {
                set |= static_cast<bool>(vector[start + j]) << j;
            }
            archive(set);
        } else {
            set_type set;
            archive(set);
            for (size_t j = 0; j < count; ++j) {
                vector[start + j] = (set & (1 << j)) > 0;
            }
        }
    }
}

template<typename T>
size_t serialization_sizeof(const std::vector<T> &vec) {
    return sizeof(size_t) + vec.size() * sizeof(typename std::vector<T>::value_type);
}

inline
size_t serialization_sizeof(const std::vector<bool> &vec) {
    using set_type = uint32_t;
    constexpr auto set_num_bits = sizeof(set_type) * 8;
    const auto num_sets = vec.size() / set_num_bits + (vec.size() % set_num_bits != 0); 
    return sizeof(size_t) + num_sets * sizeof(set_type);
}

template<typename Archive, typename T>
void serialize(Archive &archive, std::unique_ptr<T> &ptr) {
    archive(*ptr);
}

template<typename Archive, typename T>
void serialize(Archive &archive, std::shared_ptr<T> &ptr) {
    archive(*ptr);
}

}

#endif // EDYN_SERIALIZATION_STD_S11N_HPP