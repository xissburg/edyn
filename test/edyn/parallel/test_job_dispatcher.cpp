#include "../common/common.hpp"
#include "edyn/parallel/job_dispatcher.hpp"
#include "edyn/parallel/parallel_for.hpp"
#include "edyn/parallel/parallel_for_async.hpp"

#include <array>
#include <atomic>

class job_dispatcher_test: public ::testing::Test {
protected:
    void SetUp() override {
        dispatcher.start(4);
    }

    void TearDown() override {
        dispatcher.stop();
    }

public:
    edyn::job_dispatcher dispatcher;
    std::atomic<bool> done {false};
};

TEST_F(job_dispatcher_test, parallel_for) {
    constexpr size_t num_samples = 3591833;
    std::vector<int> values(num_samples);

    edyn::parallel_for(dispatcher, size_t{0}, num_samples, size_t{1}, [&](size_t i) {
        values[i] = 3;
    });

    edyn::parallel_for(dispatcher, size_t{0}, num_samples, size_t{1}, [&](size_t i) {
        values[i] = values[i] + 11;
    });

    edyn::parallel_for(dispatcher, size_t{0}, num_samples, size_t{1}, [&](size_t i) {
        ASSERT_EQ(values[i], 14);
    });
}

TEST_F(job_dispatcher_test, parallel_for_each) {
    constexpr size_t num_samples = 3591832;
    std::vector<int> values(num_samples);

    edyn::parallel_for_each(dispatcher, values.begin(), values.end(), [&](int &value) {
        value = 77;
    });

    edyn::parallel_for_each(dispatcher, values.begin(), values.end(), [&](int &value) {
        value += 14;
    });

    edyn::parallel_for_each(dispatcher, values.begin(), values.end(), [&](int &value) {
        ASSERT_EQ(value, 91);
    });
}

TEST_F(job_dispatcher_test, parallel_for_small) {
    constexpr size_t num_samples = 1139;
    std::vector<int> values(num_samples);

    edyn::parallel_for(dispatcher, size_t{0}, num_samples, size_t{1}, [&](size_t i) {
        values[i] = 27;
    });

    edyn::parallel_for(dispatcher, size_t{0}, num_samples, size_t{1}, [&](size_t i) {
        values[i] = values[i] + 18;
    });

    edyn::parallel_for(dispatcher, size_t{0}, num_samples, size_t{1}, [&](size_t i) {
        ASSERT_EQ(values[i], 27 + 18);
    });
}

TEST_F(job_dispatcher_test, parallel_for_tiny) {
    constexpr size_t num_samples = 2;
    std::vector<int> values(num_samples);

    for (auto i = 0; i < 1024; ++i) {
        edyn::parallel_for(dispatcher, size_t{0}, num_samples, size_t{1}, [&](size_t i) {
            values[i] = 27;
        });

        edyn::parallel_for(dispatcher, size_t{0}, num_samples, size_t{1}, [&](size_t i) {
            values[i] = values[i] + 18;
        });

        edyn::parallel_for(dispatcher, size_t{0}, num_samples, size_t{1}, [&](size_t i) {
            ASSERT_EQ(values[i], 27 + 18);
        });
    }
}

void parallel_for_async_completion(edyn::job::data_type &data) {
    auto archive = edyn::memory_input_archive(data.data(), data.size());
    intptr_t self_ptr;
    archive(self_ptr);
    auto self = reinterpret_cast<job_dispatcher_test *>(self_ptr);
    self->done.store(true, std::memory_order_relaxed);
}

TEST_F(job_dispatcher_test, parallel_for_async) {
    constexpr size_t num_samples = 3591833;
    std::vector<int> values(num_samples);
    done.store(false, std::memory_order_relaxed);

    // Job that will be called when the async for loop is done. It sets a
    // flag to true.
    auto completion_job = edyn::job();
    auto archive = edyn::fixed_memory_output_archive(completion_job.data.data(), completion_job.data.size());
    auto self_ptr = reinterpret_cast<intptr_t>(this);
    archive(self_ptr);
    completion_job.func = &parallel_for_async_completion;

    edyn::parallel_for_async(dispatcher, size_t{0}, num_samples, size_t{1}, completion_job, [&](size_t i) {
        values[i] = 31;
    });

    while (true) {
        if (done.load(std::memory_order_relaxed)) {
            break;
        }

        edyn::delay(11);
    }

    for (auto value : values) {
        ASSERT_EQ(value, 31);
    }
}

/*
TEST_F(job_dispatcher_test, nested_parallel_for) {
    constexpr size_t rows = 2012;
    constexpr size_t columns = 2459;
    using matrix_type = std::array<std::array<edyn::scalar, columns>, rows>;

    auto A = std::make_unique<matrix_type>();

    edyn::parallel_for(dispatcher, size_t{0}, A->size(), size_t{1}, [&](size_t i) {
        edyn::parallel_for(dispatcher, size_t{0}, (*A)[i].size(), size_t{1}, [&A, i](size_t j) {
            (*A)[i][j] = 33;
        });
    });

    edyn::parallel_for(dispatcher, size_t{0}, A->size(), size_t{1}, [&](size_t i) {
        edyn::parallel_for(dispatcher, size_t{0}, (*A)[i].size(), size_t{1}, [&A, i](size_t j) {
            (*A)[i][j] += 17;
        });
    });

    for (size_t i = 0; i < A->size(); ++i) {
        for (size_t j = 0; j < (*A)[i].size(); ++j) {
            ASSERT_EQ((*A)[i][j], 33 + 17);
        }
    }
}*/
