#include "../common/common.hpp"
#include <random>

class vector3_test: public ::testing::Test {
protected:
    std::random_device rd;
    std::mt19937 gen;
    std::uniform_real_distribution<edyn::scalar> dist;

    vector3_test() :
        gen(rd()),
        dist(-1e4, 1e4)
    {}

public:
    edyn::scalar random() {
        return dist(gen);
    }
    edyn::vector3 randomvec() {
        return {random(), random(), random()};
    }
};

TEST_F(vector3_test, fundamental) {
    auto a = randomvec();
    auto b = randomvec();

    ASSERT_SCALAR_EQ((a + b).x, a.x + b.x);
    ASSERT_SCALAR_EQ((a + b).y, a.y + b.y);
    ASSERT_SCALAR_EQ((a + b).z, a.z + b.z);

    ASSERT_SCALAR_EQ((a - b).x, a.x - b.x);
    ASSERT_SCALAR_EQ((a - b).y, a.y - b.y);
    ASSERT_SCALAR_EQ((a - b).z, a.z - b.z);

    ASSERT_SCALAR_EQ((-b).x, -b.x);
    ASSERT_SCALAR_EQ((-b).y, -b.y);
    ASSERT_SCALAR_EQ((-b).z, -b.z);

    auto s = random();

    if (s <= EDYN_EPSILON) {
        s = 1.618;
    }

    ASSERT_SCALAR_EQ((a * s).x, a.x * s);
    ASSERT_SCALAR_EQ((a * s).y, a.y * s);
    ASSERT_SCALAR_EQ((a * s).z, a.z * s);

    ASSERT_SCALAR_EQ((s * b).x, s * b.x);
    ASSERT_SCALAR_EQ((s * b).y, s * b.y);
    ASSERT_SCALAR_EQ((s * b).z, s * b.z);
    
    ASSERT_SCALAR_EQ((a / s).x, a.x / s);
    ASSERT_SCALAR_EQ((a / s).y, a.y / s);
    ASSERT_SCALAR_EQ((a / s).z, a.z / s);

    ASSERT_SCALAR_EQ((s / b).x, s / b.x);
    ASSERT_SCALAR_EQ((s / b).y, s / b.y);
    ASSERT_SCALAR_EQ((s / b).z, s / b.z);

    edyn::vector3 c {a.x + b.x, a.y + b.y, a.z + b.z};
    a += b;
    ASSERT_SCALAR_EQ(a.x, c.x);
    ASSERT_SCALAR_EQ(a.y, c.y);
    ASSERT_SCALAR_EQ(a.z, c.z);
    
    edyn::vector3 d {b.x - a.x, b.y - a.y, b.z - a.z};
    b -= a;
    ASSERT_SCALAR_EQ(b.x, d.x);
    ASSERT_SCALAR_EQ(b.y, d.y);
    ASSERT_SCALAR_EQ(b.z, d.z);
}

TEST_F(vector3_test, dot) {
    auto a = randomvec();
    auto b = randomvec();
    auto x = edyn::dot(a, b);
    auto y = a.x * b.x + a.y * b.y + a.z * b.z;
    ASSERT_SCALAR_EQ(x, y);
}

TEST_F(vector3_test, cross) {
    constexpr auto vx = edyn::vector3_x;
    constexpr auto vy = edyn::vector3_y;
    constexpr auto vz = edyn::vector3_z;

    ASSERT_EQ(edyn::cross(vx, vx), edyn::vector3_zero);
    ASSERT_EQ(edyn::cross(vx, -vx), edyn::vector3_zero);
    ASSERT_EQ(edyn::cross(vx, vy), vz);
    ASSERT_EQ(edyn::cross(vz, vx), vy);
    ASSERT_EQ(edyn::cross(vy, vz), vx);

    auto a = edyn::normalize(randomvec());
    auto b = edyn::normalize(randomvec());
    auto c = edyn::cross(a, b);

    // Orthogonal (-ish)
    ASSERT_NEAR(edyn::dot(a, c), 0, 1e-4);
    ASSERT_NEAR(edyn::dot(b, c), 0, 1e-4);
}

TEST_F(vector3_test, length) {
    auto v = randomvec();
    auto l2 = edyn::length_sqr(v);
    ASSERT_SCALAR_EQ(l2, v.x * v.x + v.y * v.y + v.z * v.z);
    auto l = edyn::length(v);
    ASSERT_SCALAR_EQ(l, std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z));
}

TEST_F(vector3_test, normalize) {
    auto v = randomvec();

    if (edyn::length_sqr(v) < EDYN_EPSILON) {
        v = edyn::vector3_x;
    }

    auto nv = edyn::normalize(v);
    auto l = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
    ASSERT_SCALAR_EQ(nv.x, v.x / l);
    ASSERT_SCALAR_EQ(nv.y, v.y / l);
    ASSERT_SCALAR_EQ(nv.z, v.z / l);
}

TEST_F(vector3_test, comparison) {
    auto v = randomvec();
    auto w = randomvec();
    ASSERT_TRUE(v == v);
    ASSERT_FALSE(v == w);
    ASSERT_FALSE(v != v);
    ASSERT_TRUE(v != w);

    auto r = edyn::vector3{1, 2, 3};
    ASSERT_GT(r, edyn::vector3_zero);
    
    auto s = edyn::vector3{0, -1, 2.99};
    ASSERT_LT(s, r);
}