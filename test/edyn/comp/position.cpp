#include "../common/common.hpp"
#include <edyn/comp/position.hpp>

TEST(position, vector_conversion) {
    edyn::position pos {2, 3 ,4};
    edyn::vector3 v {-2, -4, -3};

    ASSERT_EQ(pos + v, pos.v + v);
    ASSERT_EQ(v - pos, v - pos.v);

    edyn::position pos1 { pos.v.x - v.x, pos.v.y - v.y, pos.v.z - v.z};
    pos -= v;
    ASSERT_EQ(pos, pos1);

    edyn::scalar s {3.14};
    edyn::position pos2 { pos.v.x * s, pos.v.y * s, pos.v.z * s};
    ASSERT_EQ(pos * s, pos2);
    ASSERT_EQ(s * pos, pos2);
    pos *= s;
    ASSERT_EQ(pos, pos2);
}