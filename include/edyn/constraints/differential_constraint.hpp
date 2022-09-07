#ifndef EDYN_CONSTRAINTS_DIFFERENTIAL_CONSTRAINT_HPP
#define EDYN_CONSTRAINTS_DIFFERENTIAL_CONSTRAINT_HPP

#include <array>
#include <entt/entity/fwd.hpp>
#include <entt/entity/entity.hpp>
#include "edyn/math/scalar.hpp"

namespace edyn {

struct vector3;
struct quaternion;
struct constraint_row_prep_cache;

struct differential_constraint {
    std::array<entt::entity, 3> body {entt::null, entt::null, entt::null};
    scalar ratio;
    scalar impulse;

    void prepare(
        const entt::registry &, entt::entity,
        constraint_row_prep_cache &cache, scalar dt,
        const vector3 &originA, const vector3 &posA, const quaternion &ornA,
        const vector3 &linvelA, const vector3 &angvelA,
        const vector3 &originB, const vector3 &posB, const quaternion &ornB,
        const vector3 &linvelB, const vector3 &angvelB,
        const vector3 &originC, const vector3 &posC, const quaternion &ornC,
        const vector3 &linvelC, const vector3 &angvelC);
};

template<typename Archive>
void serialize(Archive &archive, differential_constraint &con) {
    archive(con.body);
    archive(con.ratio);
    archive(con.impulse);
}

}

#endif // EDYN_CONSTRAINTS_DIFFERENTIAL_CONSTRAINT_HPP
