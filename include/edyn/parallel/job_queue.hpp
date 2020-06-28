#ifndef EDYN_PARALLEL_JOB_QUEUE_HPP
#define EDYN_PARALLEL_JOB_QUEUE_HPP

#include <mutex>
#include <atomic>
#include <vector>
#include <memory>
#include <functional>
#include <condition_variable>

namespace edyn {

class job {
public:
    virtual ~job() {}

    virtual void run() = 0;

    void operator()();
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

class job_queue {
public:
    void push(std::shared_ptr<job> j);

    std::shared_ptr<job> pop();

    std::shared_ptr<job> try_pop();

    void unblock();

    size_t size();

private:
    std::mutex m_mutex;
    std::vector<std::shared_ptr<job>> m_jobs;
    std::condition_variable m_cv;
    std::atomic_bool m_unblock {false};
};

}

#endif // EDYN_PARALLEL_JOB_QUEUE_HPP