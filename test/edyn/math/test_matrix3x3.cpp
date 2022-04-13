#include "../common/common.hpp"
#include <random>

class matrix3x3_test: public ::testing::Test {
protected:
    std::random_device rd;
    std::mt19937 gen;
    std::uniform_real_distribution<edyn::scalar> dist;

    matrix3x3_test() :
        gen(rd()),
        dist(-1e4, 1e4)
    {}

public:
    edyn::scalar random() {
        return dist(gen);
    }
    edyn::matrix3x3 random_mat() {
        return {
            edyn::vector3{random(), random(), random()},
            edyn::vector3{random(), random(), random()},
            edyn::vector3{random(), random(), random()}
        };
    }
};

TEST_F(matrix3x3_test, fundamental) {
    auto m = random_mat();
    auto n = random_mat();
    auto p = m + n;

    for (auto i = 0; i < 3; ++i) {
        for (auto j = 0; j < 3; ++j) {
            ASSERT_SCALAR_EQ(p[i][j], m[i][j] + n[i][j]);
        }
    }

    m += n;

    for (auto i = 0; i < 3; ++i) {
        for (auto j = 0; j < 3; ++j) {
            ASSERT_SCALAR_EQ(m[i][j], p[i][j]);
        }
    }
}

TEST_F(matrix3x3_test, comparison) {
    auto m = random_mat();
    ASSERT_EQ(m, m);

    auto n = m;
    n[0][0] += 1;
    ASSERT_NE(n, m);
}
