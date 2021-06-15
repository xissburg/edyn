#include "../common/common.hpp"
#include "edyn/math/math.hpp"

TEST(math_test, average) {
    auto vertices = std::array<edyn::vector3, 3>{
        edyn::vector3{1, 1, 0},
        edyn::vector3{-1, 1, 0},
        edyn::vector3{0, -1, 0}
    };
    auto center = edyn::average(vertices);
    ASSERT_SCALAR_EQ(center.x, 0);
    ASSERT_SCALAR_EQ(center.y, 1.f / 3.f);
    ASSERT_SCALAR_EQ(center.z, 0);
}
