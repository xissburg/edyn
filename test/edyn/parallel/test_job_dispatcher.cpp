#include "../common/common.hpp"

#include <array>
#include <future>

class job_dispatcher_test: public ::testing::Test {
protected:
    void SetUp() override {
        dispatcher.start();
    }

    void TearDown() override {
        dispatcher.stop();
    }

    edyn::job_dispatcher dispatcher;
};

TEST_F(job_dispatcher_test, parallel_for) {
    constexpr size_t num_samples = 3591833;
    std::vector<int> values(num_samples);

    edyn::parallel_for(dispatcher, size_t{0}, num_samples, size_t{1}, [&] (size_t i) {
        values[i] = 3;
    });

    edyn::parallel_for(dispatcher, size_t{0}, num_samples, size_t{1}, [&] (size_t i) {
        values[i] = values[i] + 11;
    });

    edyn::parallel_for(dispatcher, size_t{0}, num_samples, size_t{1}, [&] (size_t i) {
        ASSERT_EQ(values[i], 14);
    });
}

TEST_F(job_dispatcher_test, parallel_for_each) {
    constexpr size_t num_samples = 3591832;
    std::vector<int> values(num_samples);

    edyn::parallel_for_each(dispatcher, values.begin(), values.end(), [&] (auto it) {
        *it = 77;
    });

    edyn::parallel_for_each(dispatcher, values.begin(), values.end(), [&] (auto it) {
        *it += 14;
    });

    edyn::parallel_for_each(dispatcher, values.begin(), values.end(), [&] (auto it) {
        ASSERT_EQ(*it, 91);
    });
}

TEST_F(job_dispatcher_test, parallel_for_small) {
    constexpr size_t num_samples = 1139;
    std::vector<int> values(num_samples);

    edyn::parallel_for(dispatcher, size_t{0}, num_samples, size_t{1}, [&] (size_t i) {
        values[i] = 27;
    });

    edyn::parallel_for(dispatcher, size_t{0}, num_samples, size_t{1}, [&] (size_t i) {
        values[i] = values[i] + 18;
    });

    edyn::parallel_for(dispatcher, size_t{0}, num_samples, size_t{1}, [&] (size_t i) {
        ASSERT_EQ(values[i], 27 + 18);
    });
}

TEST_F(job_dispatcher_test, nested_parallel_for) {
    constexpr size_t rows = 2012;
    constexpr size_t columns = 2459;
    using matrix_type = std::array<std::array<edyn::scalar, columns>, rows>;

    auto A = std::make_unique<matrix_type>();

    edyn::parallel_for(dispatcher, size_t{0}, A->size(), size_t{1}, [&] (size_t i) {
        edyn::parallel_for(dispatcher, size_t{0}, (*A)[i].size(), size_t{1}, [&A, i] (size_t j) {
            (*A)[i][j] = 33;
        });
    });

    edyn::parallel_for(dispatcher, size_t{0}, A->size(), size_t{1}, [&] (size_t i) {
        edyn::parallel_for(dispatcher, size_t{0}, (*A)[i].size(), size_t{1}, [&A, i] (size_t j) {
            (*A)[i][j] += 17;
        });
    });

    for (size_t i = 0; i < A->size(); ++i) {
        for (size_t j = 0; j < (*A)[i].size(); ++j) {
            ASSERT_EQ((*A)[i][j], 33 + 17);
        }
    }
}