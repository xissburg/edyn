#ifndef EDYN_SERIALIZATION_S11N_UTIL_HPP
#define EDYN_SERIALIZATION_S11N_UTIL_HPP

#include <tuple>

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
    long long,
    unsigned long,
    unsigned long long,
    float,
    double
>;

template<typename Archive, typename T>
void serialize(Archive &, T &);

}

#endif // EDYN_SERIALIZATION_S11N_UTIL_HPP