#ifndef EDYN_PARALLEL_CONTACT_EVENT_HPP
#define EDYN_PARALLEL_CONTACT_EVENT_HPP

namespace edyn {

struct contact_started {
    entt::entity manifold_entity;
};

}

#endif // EDYN_PARALLEL_CONTACT_EVENT_HPP
