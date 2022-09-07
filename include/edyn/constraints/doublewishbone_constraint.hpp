#ifndef EDYN_CONSTRAINTS_DOUBLEWISHBONE_CONSTRAINT
#define EDYN_CONSTRAINTS_DOUBLEWISHBONE_CONSTRAINT

#include <entt/fwd.hpp>
#include "edyn/math/vector3.hpp"
#include "edyn/constraints/constraint_base.hpp"
#include "edyn/util/array_util.hpp"

namespace edyn {

struct quaternion;
struct constraint_row_prep_cache;

struct doublewishbone_constraint : public constraint_base {
    vector3 upper_pivotA;
    vector3 upper_pivotB;
    scalar upper_length;
    vector3 lower_pivotA;
    vector3 lower_pivotB;
    scalar lower_length;
    bool steerable;

    static const auto num_rows = 6;
    std::array<scalar, num_rows> impulse = make_array<num_rows>(scalar{});

    void prepare(
        const entt::registry &, entt::entity,
        constraint_row_prep_cache &cache, scalar dt,
        const vector3 &originA, const vector3 &posA, const quaternion &ornA,
        const vector3 &linvelA, const vector3 &angvelA,
        const vector3 &originB, const vector3 &posB, const quaternion &ornB,
        const vector3 &linvelB, const vector3 &angvelB);
};

template<typename Archive>
void serialize(Archive &archive, doublewishbone_constraint &con) {
    archive(con.body);
    archive(con.upper_pivotA);
    archive(con.upper_pivotB);
    archive(con.upper_length);
    archive(con.lower_pivotA);
    archive(con.lower_pivotB);
    archive(con.lower_length);
    archive(con.steerable);
    archive(con.impulse);
}

}

#endif // EDYN_CONSTRAINTS_DOUBLEWISHBONE_CONSTRAINT
