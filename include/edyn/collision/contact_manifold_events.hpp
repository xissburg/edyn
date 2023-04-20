#ifndef EDYN_COLLISION_CONTACT_MANIFOLD_EVENTS_HPP
#define EDYN_COLLISION_CONTACT_MANIFOLD_EVENTS_HPP

#include <array>
#include "edyn/config/constants.hpp"

namespace edyn {

struct contact_manifold_events {
    bool contact_started {false};
    bool contact_ended {false};
    unsigned num_contacts_created {0};
    std::array<unsigned, max_contacts> contacts_created;
    unsigned num_contacts_destroyed {0};
    std::array<unsigned, max_contacts> contacts_destroyed;
};

}

#endif // EDYN_COLLISION_CONTACT_MANIFOLD_EVENTS_HPP
