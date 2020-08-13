#ifndef EDYN_SERIALIZATION_S11N_UTIL_HPP
#define EDYN_SERIALIZATION_S11N_UTIL_HPP

namespace edyn {

using archive_fundamental_types = std::tuple<
    bool,
    char,
    unsigned char,
    short,
    unsigned short,
    int,
    unsigned int,
    long,
    unsigned long,
    float,
    double
>;


template <typename T, typename Tuple>
struct has_type;

template <typename T, typename... Us>
struct has_type<T, std::tuple<Us...>> : std::disjunction<std::is_same<T, Us>...> {};

template<typename Archive, typename T>
void serialize(Archive &, T &);

}

#endif // EDYN_SERIALIZATION_S11N_UTIL_HPP