#ifndef EDYN_COMP_TAG_HPP
#define EDYN_COMP_TAG_HPP

namespace edyn {

/**
 * Rigid body.
 */
struct rigidbody_tag {};

/**
 * Identifies a constraint.
 */
struct constraint_tag {};

/**
 * An island.
 */
struct island_tag {};

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
 * Procedural entity, which is an entity that has calculated state and can only
 * be present in a single island.
 */
struct procedural_tag {};

/**
 * An entity that is synchronized between client and server.
 */
struct networked_tag {};

/**
 * An entity that should not be shared between client and server.
 */
struct network_exclude_tag {};

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

}

#endif // EDYN_COMP_TAG_HPP
