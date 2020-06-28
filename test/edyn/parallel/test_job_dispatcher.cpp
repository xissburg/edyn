#include "../common/common.hpp"

#include <array>

struct nop_job: public edyn::job {
    int m_i {0};

    void run() override {
        ++m_i;
    }
};

TEST(job_dispatcher_test, async) {
    auto dispatcher = edyn::job_dispatcher();
    dispatcher.start();

    auto job0 = std::make_shared<nop_job>();
    auto job1 = std::make_shared<nop_job>();

    dispatcher.async(job0);
    dispatcher.async(job1);

    job0->join();
    job1->join();

    ASSERT_EQ(job0->m_i, 1);
    ASSERT_EQ(job1->m_i, 1);
}

TEST(job_dispatcher_test, parallel_for) {
    auto dispatcher = edyn::job_dispatcher();
    dispatcher.start();

    constexpr size_t num_samples = 3591833;
    std::vector<edyn::scalar> radians(num_samples);
    std::vector<edyn::scalar> cosines(num_samples);

    edyn::parallel_for(dispatcher, size_t{0}, num_samples, size_t{1}, [&] (size_t i) {
        auto unit = edyn::scalar(i) - edyn::scalar(num_samples) * edyn::scalar(0.5);
        radians[i] = unit * edyn::pi;
        cosines[i] = std::cos(radians[i]);
    });

    ASSERT_SCALAR_EQ(cosines[45], std::cos(radians[45]));
    ASSERT_SCALAR_EQ(cosines[5095], std::cos(radians[5095]));
    ASSERT_SCALAR_EQ(cosines[2990190], std::cos(radians[2990190]));
    ASSERT_SCALAR_EQ(cosines[num_samples - 1], std::cos(radians[num_samples - 1]));
}