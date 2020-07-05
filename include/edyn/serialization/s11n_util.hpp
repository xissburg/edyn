#ifndef EDYN_SERIALIZATION_S11N_UTIL_HPP
#define EDYN_SERIALIZATION_S11N_UTIL_HPP

namespace edyn {

template<typename Archive, typename T>
void serialize(Archive &, T &);

}

#endif // EDYN_SERIALIZATION_S11N_UTIL_HPP