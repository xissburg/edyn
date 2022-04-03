#ifndef EDYN_COMP_TAG_HPP
#define EDYN_COMP_TAG_HPP

namespace edyn {

/**
 * Rigid body.
 */
struct rigidbody_tag {};

/**
 * Dynamic rigid body.
 */
struct dynamic_tag {};

/**
 * Kinematic rigid body.
 */
struct kinematic_tag {};

/**
 * Static rigid body.
 */
struct static_tag {};

/**
 * Procedural entity, i.e. an entity that has calculated state.
 */
struct procedural_tag {};

/**
 * An entity that is synchronized between client and server.
 */
struct networked_tag {};

/**
 * An entity that is currently asleep.
 */
struct sleeping_tag {};

/**
 * An entity that won't be put to sleep when inactive.
 */
struct sleeping_disabled_tag {};

/**
 * Disabled entity.
 */
struct disabled_tag {};

/**
 * A rigid body which will have its contact state continuously updated in the
 * main registry.
 */
struct continuous_contacts_tag {};

/**
 * A rigid body which holds a shape that can roll. An extra step will be
 * performed to help improve contact point persistence when rolling at
 * higher speeds.
 */
struct rolling_tag {};

/**
 * An entity that was created externally and tagged via
 * `edyn::tag_external_entity` (i.e. it doesn't represent any of the internal
 * Edyn entity types such as rigid bodies and constraints) and is part of the
 * entity graph and can be attached to an internal Edyn entity by means of a
 * `edyn::null_constraint`.
 */
struct external_tag {};

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
void serialize(Archive &, rolling_tag &) {}

template<typename Archive>
void serialize(Archive &, external_tag &) {}

}

#endif // EDYN_COMP_TAG_HPP
