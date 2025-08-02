#include "../common/common.hpp"
#include "edyn/context/task.hpp"
#include "edyn/parallel/job_dispatcher.hpp"

#include <array>
#include <atomic>

class job_dispatcher_test: public ::testing::Test {
protected:
    void SetUp() override {
        edyn::job_dispatcher::global().start(4);
    }

    void TearDown() override {
        edyn::job_dispatcher::global().stop();
    }

public:
    std::atomic<bool> done {false};
};

template<typename Func>
void run_task_wait(Func task_func, unsigned size) {
    auto task = edyn::task_delegate_t(entt::connect_arg_t<&decltype(task_func)::operator()>{}, task_func);
    edyn::enqueue_task_wait_default(task, size);
}

TEST_F(job_dispatcher_test, parallel_for) {
    constexpr size_t num_samples = 3591833;
    std::vector<int> values(num_samples);

    run_task_wait([&](unsigned start, unsigned end) {
        for (auto i = start; i < end; ++i) {
            values[i] = 3;
        }
    }, num_samples);

    run_task_wait([&](unsigned start, unsigned end) {
        for (auto i = start; i < end; ++i) {
            values[i] += 11;
        }
    }, num_samples);

    for (auto value : values) {
        ASSERT_EQ(value, 14);
    }
}

TEST_F(job_dispatcher_test, parallel_for_small) {
    constexpr size_t num_samples = 1139;
    std::vector<int> values(num_samples);

    run_task_wait([&](unsigned start, unsigned end) {
        for (auto i = start; i < end; ++i) {
            values[i] = 27;
        }
    }, num_samples);

    run_task_wait([&](unsigned start, unsigned end) {
        for (auto i = start; i < end; ++i) {
            values[i] = values[i] + 18;
        }
    }, num_samples);

    for (auto value : values) {
        ASSERT_EQ(value, 27 + 18);
    }
}

TEST_F(job_dispatcher_test, parallel_for_tiny) {
    constexpr size_t num_samples = 2;
    std::vector<int> values(num_samples);

    for (auto i = 0; i < 1024; ++i) {
        run_task_wait([&](unsigned start, unsigned end) {
            for (auto i = start; i < end; ++i) {
                values[i] = 27;
            }
        }, num_samples);

        run_task_wait([&](unsigned start, unsigned end) {
            for (auto i = start; i < end; ++i) {
                values[i] = values[i] + 18;
            }
        }, num_samples);

        for (auto value : values) {
            ASSERT_EQ(value, 27 + 18);
        }
    }
}

void parallel_for_async_completion(job_dispatcher_test &self) {
    self.done.store(true, std::memory_order_relaxed);
}

TEST_F(job_dispatcher_test, parallel_for_async) {
    constexpr size_t num_samples = 3591833;
    std::vector<int> values(num_samples);
    done.store(false, std::memory_order_relaxed);

    auto task_func = [&](unsigned start, unsigned end) {
        for (auto i = start; i < end; ++i) {
            values[i] = 31;
        }
    };
    auto task = edyn::task_delegate_t(entt::connect_arg_t<&decltype(task_func)::operator()>{}, task_func);
    auto completion = edyn::task_completion_delegate_t(entt::connect_arg_t<&parallel_for_async_completion>{}, *this);
    edyn::enqueue_task_default(task, num_samples, completion);

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
