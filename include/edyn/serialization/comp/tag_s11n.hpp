#ifndef EDYN_SERIALIZATION_COMP_TAG_S11N_HPP
#define EDYN_SERIALIZATION_COMP_TAG_S11N_HPP

#include "edyn/comp/tag.hpp"

namespace edyn {

template<typename Archive>
void serialize(Archive &, rigidbody_tag &) {}

template<typename Archive>
void serialize(Archive &, dynamic_tag &) {}

template<typename Archive>
void serialize(Archive &, kinematic_tag &) {}

template<typename Archive>
void serialize(Archive &, static_tag &) {}

template<typename Archive>
void serialize(Archive &, procedural_tag &) {}

template<typename Archive>
void serialize(Archive &, sleeping_tag &) {}

template<typename Archive>
void serialize(Archive &, sleeping_disabled_tag &) {}

template<typename Archive>
void serialize(Archive &, disabled_tag &) {}

template<typename Archive>
void serialize(Archive &, continuous_contacts_tag &) {}

template<typename Archive>
void serialize(Archive &, external_tag &) {}

}

#endif // EDYN_SERIALIZATION_COMP_TAG_S11N_HPP