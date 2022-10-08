#ifndef EDYN_PARALLEL_ATOMIC_COUNTER_SYNC_HPP
#define EDYN_PARALLEL_ATOMIC_COUNTER_SYNC_HPP

#include "edyn/config/config.h"
#include <mutex>
#include <condition_variable>

namespace edyn {

class atomic_counter_sync {
public:
    atomic_counter_sync(size_t count)
        : count(count)
        , done(false)
    {}

    ~atomic_counter_sync() {
        EDYN_ASSERT(count == 0);
    }

    void decrement() {
        std::lock_guard lock(mutex);
        EDYN_ASSERT(count > 0);
        --count;
        done = count == 0;

        if (done) {
            cv.notify_one();
        }
    }

    void wait() {
        std::unique_lock lock(mutex);
        cv.wait(lock, [&] { return done; });
    }

private:
    size_t count;
    bool done;
    std::mutex mutex;
    std::condition_variable cv;
};

}

#endif // EDYN_PARALLEL_ATOMIC_COUNTER_SYNC_HPP
