#ifndef EDYN_PARALLEL_JOB_HPP
#define EDYN_PARALLEL_JOB_HPP

#include <array>
#include <cstddef>
#include <cstdint>

namespace edyn {

struct job {
    static constexpr size_t pointer_size = sizeof(void(*)(void));
    static constexpr size_t size = 64;
    static constexpr size_t data_size = size - pointer_size;

    using data_type = std::array<uint8_t, data_size>;
    using function_type = void(data_type &);

    data_type data;
    function_type *func;

    void operator()() {
        (*func)(data);
    }

    static auto noop() {
        job j;
        j.func = [] (data_type &) {};
        return j;
    }
};

}

#endif // EDYN_PARALLEL_JOB_HPP