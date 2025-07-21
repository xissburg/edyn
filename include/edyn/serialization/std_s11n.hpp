#ifndef EDYN_SERIALIZATION_STD_S11N_HPP
#define EDYN_SERIALIZATION_STD_S11N_HPP

#include <array>
#include <limits>
#include <map>
#include <vector>
#include <cstdint>
#include <variant>
#include <string>
#include <utility>
#include <optional>
#include <type_traits>
#include <entt/core/ident.hpp>
#include "edyn/util/tuple_util.hpp"

namespace edyn {

template<typename Archive>
void serialize(Archive &archive, std::string& str) {
    using size_type = uint16_t;
    size_type size = std::min(str.size(), static_cast<size_t>(std::numeric_limits<size_type>::max()));
    archive(size);
    str.resize(size);

    for (size_t i = 0; i < size; ++i) {
        archive(str[i]);
    }
}

template<typename Archive, typename T>
void serialize(Archive &archive, std::vector<T> &vector) {
    using size_type = uint16_t;
    size_type size = std::min(vector.size(), static_cast<size_t>(std::numeric_limits<size_type>::max()));
    archive(size);
    vector.resize(size);

    for (size_t i = 0; i < size; ++i) {
        archive(vector[i]);
    }
}

template<typename Archive>
void serialize(Archive &archive, std::vector<bool> &vector) {
    using size_type = uint16_t;
    size_type size = std::min(vector.size(), static_cast<size_t>(std::numeric_limits<size_type>::max()));
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
    return sizeof(uint16_t) + vec.size() * sizeof(typename std::vector<T>::value_type);
}

inline
size_t serialization_sizeof(const std::vector<bool> &vec) {
    using set_type = uint32_t;
    constexpr auto set_num_bits = sizeof(set_type) * 8;
    const auto num_sets = vec.size() / set_num_bits + (vec.size() % set_num_bits != 0);
    return sizeof(uint16_t) + num_sets * sizeof(set_type);
}

template<typename Archive, typename T, size_t N>
void serialize(Archive &archive, std::array<T, N> &arr) {
    for (size_t i = 0; i < arr.size(); ++i) {
        archive(arr[i]);
    }
}

template<typename T, size_t N>
constexpr size_t serialization_sizeof(const std::array<T, N> &arr) {
    return sizeof(T) * N;
}

namespace internal {
    template<typename T, typename Archive, typename... Ts>
    void read_variant(Archive& archive, std::variant<Ts...>& var) {
        auto t = T{};
        archive(t);
        var = std::variant<Ts...>{t};
    }

    template<typename Archive, typename IDType, typename... Ts, IDType... Indexes>
    void read_variant(Archive& archive, IDType id, std::variant<Ts...>& var, std::integer_sequence<IDType, Indexes...>)
    {
        ((id == Indexes ? read_variant<std::tuple_element_t<Indexes, std::tuple<Ts...>>>(archive, var) : (void)0), ...);
    }

    template<typename Archive, typename IDType, typename... Ts>
    void read_variant(Archive& archive, IDType id, std::variant<Ts...>& var)
    {
        read_variant(archive, id, var, std::make_integer_sequence<IDType, sizeof...(Ts)>{});
    }
}

template<typename Archive, typename... Ts>
void serialize(Archive& archive, std::variant<Ts...>& var) {
    using id_type = uint8_t;

    if constexpr(Archive::is_input::value) {
        id_type id {};
        archive(id);
        internal::read_variant(archive, id, var);
    } else {
        std::visit([&archive](auto &&t) {
            using T = std::decay_t<decltype(t)>;
            auto id = index_of_v<id_type, T, Ts...>;
            archive(id);
            archive(t);
        }, var);
    }
}

template<typename... Ts>
size_t serialization_sizeof(const std::variant<Ts...>& var) {
    size_t content_size;
    std::visit([&content_size](auto &&t) {
        content_size = sizeof(t);
    });
    return sizeof(uint8_t) + content_size;
}

template<typename Archive, typename T, typename U>
void serialize(Archive &archive, std::pair<T, U> &pair) {
    archive(pair.first);
    archive(pair.second);
}

template<typename T, typename U>
constexpr size_t serialization_sizeof(const std::pair<T, U> &) {
    return sizeof(T) + sizeof(U);
}

template<typename Archive, typename T>
void serialize(Archive &archive, std::optional<T> &opt) {
    bool has_value = opt.has_value();
    archive(has_value);

    if constexpr(Archive::is_input::value) {
        if (has_value) {
            archive(opt.emplace());
        } else {
            opt.reset();
        }
    } else {
        if (has_value) {
            archive(*opt);
        }
    }
}

template<typename T>
size_t serialization_sizeof(const std::optional<T> &opt) {
    if (opt) {
        return sizeof(bool) + serialization_sizeof(*opt);
    }

    return sizeof(bool);
}

template<typename Archive, typename K, typename V>
void serialize(Archive &archive, std::map<K, V> &map) {
    using size_type = uint16_t;
    size_type size = std::min(map.size(), static_cast<size_t>(std::numeric_limits<size_type>::max()));
    archive(size);

    if constexpr(Archive::is_input::value) {
        auto pair = std::pair<K, V>{};
        for (size_type i = 0; i < size; ++i) {
            archive(pair);
            map.emplace(pair);
        }
    } else {
        for (auto &pair : map) {
            // Safe to const_cast since it's an output archive thus will only
            // read from arguments.
            auto &first = const_cast<K &>(pair.first);
            archive(first);
            archive(pair.second);
        }
    }
}

template<typename K, typename V>
size_t serialization_sizeof(const std::map<K, V> &map) {
    using size_type = uint16_t;
    return sizeof(size_type) + map.size() * (sizeof(K) + sizeof(V));
}

}

#endif // EDYN_SERIALIZATION_STD_S11N_HPP
