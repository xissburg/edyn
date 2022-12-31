#ifndef EDYN_COMP_CHILD_LIST_HPP
#define EDYN_COMP_CHILD_LIST_HPP

#include <entt/entity/fwd.hpp>
#include <entt/entity/entity.hpp>

namespace edyn {

struct parent_comp {
    entt::entity child {entt::null};
};

struct child_list {
    entt::entity parent {entt::null};
    entt::entity next {entt::null};
};

template<typename Archive>
void serialize(Archive &archive, parent_comp &par) {
    archive(par.child);
}

template<typename Archive>
void serialize(Archive &archive, child_list &child) {
    archive(child.parent);
    archive(child.next);
}

}

#endif // EDYN_COMP_CHILD_LIST_HPP
