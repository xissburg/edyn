#ifndef EDYN_PARALLEL_JOB_QUEUE_HPP
#define EDYN_PARALLEL_JOB_QUEUE_HPP

#include <mutex>
#include <vector>
#include <functional>
#include <condition_variable>

namespace edyn {

class job {
public:
    job(const std::function<void(void)> &f)
        : function(f)
        , m_cv(std::make_unique<std::condition_variable>())
        , m_mutex(std::make_unique<std::mutex>())
        , m_done(false)
    {}

    void operator()() {
        std::lock_guard<std::mutex> lock(*m_mutex);
        EDYN_ASSERT(!m_done);
        function();
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
    std::function<void(void)> function;
    std::unique_ptr<std::condition_variable> m_cv;
    std::unique_ptr<std::mutex> m_mutex;
    bool m_done;
};

class job_queue {
public:
    void push(job &j) {
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            m_jobs.push_back(std::move(j));
        }
        m_cv.notify_one();
    }

    job pop() {
        std::unique_lock<std::mutex> lock(m_mutex);
        m_cv.wait(lock, [&] () {
            return !m_jobs.empty();
        });

        auto j = std::move(m_jobs.back());
        m_jobs.pop_back();
        return j;
    }

    bool maybe_pop(job &j) {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (m_jobs.empty()) {
            return false;
        }

        j = std::move(m_jobs.back());
        m_jobs.pop_back();
        return true;
    }

    size_t size() {
        std::lock_guard<std::mutex> lock(m_mutex);
        return m_jobs.size();
    }

private:
    std::mutex m_mutex;
    std::vector<job> m_jobs;
    std::condition_variable m_cv;
};

}

#endif // EDYN_PARALLEL_JOB_QUEUE_HPP