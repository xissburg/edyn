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

struct nop_job: public edyn::job {
    int m_i {0};
    std::promise<void> m_promise;

    void run() override {
        ++m_i;
        m_promise.set_value();
    }

    auto join() { 
        m_promise.get_future().get();
    }
};

TEST_F(job_dispatcher_test, async) {
    auto job0 = std::make_shared<nop_job>();
    auto job1 = std::make_shared<nop_job>();

    dispatcher.async(job0);
    dispatcher.async(job1);

    job0->join();
    job1->join();

    ASSERT_EQ(job0->m_i, 1);
    ASSERT_EQ(job1->m_i, 1);
}

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