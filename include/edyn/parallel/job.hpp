#ifndef EDYN_PARALLEL_JOB_HPP
#define EDYN_PARALLEL_JOB_HPP

#include <functional>

namespace edyn {

class job {
public:
    virtual ~job() {}

    virtual void run() = 0;
};

class std_function_job : public job {
public:
    std_function_job(const std::function<void(void)> &f)
        : m_function(f)
    {}

    void run() override {
        m_function();
    }

private:
    std::function<void(void)> m_function;
};

}

#endif // EDYN_PARALLEL_JOB_HPP