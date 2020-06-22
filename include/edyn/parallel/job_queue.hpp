#ifndef EDYN_PARALLEL_JOB_QUEUE_HPP
#define EDYN_PARALLEL_JOB_QUEUE_HPP

#include <mutex>
#include <vector>
#include <memory>
#include <functional>
#include <condition_variable>

namespace edyn {

class job {
public:
    job()
        : m_cv(std::make_unique<std::condition_variable>())
        , m_mutex(std::make_unique<std::mutex>())
        , m_done(false)
    {}

    virtual void run() = 0;

    void operator()() {
        std::lock_guard<std::mutex> lock(*m_mutex);
        EDYN_ASSERT(!m_done);
        run();
        m_done = true;
        m_cv->notify_one();
    }

    void join() {
        std::unique_lock<std::mutex> lock(*m_mutex);
        m_cv->wait(lock, [&] () {
            return m_done;
        });
    }

private:
    std::unique_ptr<std::condition_variable> m_cv;
    std::unique_ptr<std::mutex> m_mutex;
    bool m_done;
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
    void push(std::shared_ptr<job> j) {
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            m_jobs.push_back(j);
        }
        m_cv.notify_one();
    }

    std::shared_ptr<job> pop() {
        std::unique_lock<std::mutex> lock(m_mutex);
        m_cv.wait(lock, [&] () {
            return !m_jobs.empty();
        });

        auto j = m_jobs.back();
        m_jobs.pop_back();
        return j;
    }

    std::shared_ptr<job> try_pop() {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (m_jobs.empty()) {
            return {};
        }

        auto j = m_jobs.back();
        m_jobs.pop_back();
        return j;
    }

    size_t size() {
        std::lock_guard<std::mutex> lock(m_mutex);
        return m_jobs.size();
    }

private:
    std::mutex m_mutex;
    std::vector<std::shared_ptr<job>> m_jobs;
    std::condition_variable m_cv;
};

}

#endif // EDYN_PARALLEL_JOB_QUEUE_HPP