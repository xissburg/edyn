#ifndef EDYN_NETWORKING_COMP_NETWORK_DIRTY_HPP
#define EDYN_NETWORKING_COMP_NETWORK_DIRTY_HPP

#include <array>
#include <entt/core/type_info.hpp>
#include "edyn/config/config.h"

namespace edyn {

struct network_dirty {
    struct entry {
        entt::id_type type_id;
        double time; // The time this type was marked as dirty.
    };

    using size_type = uint8_t;
    static constexpr size_type max_size = 16;
    std::array<entry, max_size> entries;
    size_type size {0};

    void insert(entt::id_type type_id, double time) {
        for (unsigned i = 0; i < size; ++i) {
            if (entries[i].type_id == type_id) {
                entries[i].time = time;
                return;
            }
        }

        EDYN_ASSERT(size < max_size);
        entries[size++] = {type_id, time};
    }

    template<typename T>
    void insert(double time) {
        auto type_id = entt::type_index<T>::value();
        insert(type_id, time);
    }

    void expire(double current_time, double expiry_duration) {
        for (unsigned i = 0; i < size;) {
            if (entries[i].time + expiry_duration < current_time) {
                entries[i] = entries[--size];
            } else {
                ++i;
            }
        }
    }

    bool empty() const {
        return size == 0;
    }

    template<typename Func>
    void erase_if(Func predicate) {
        for (unsigned i = 0; i < size;) {
            if (predicate(entries[i].type_id)) {
                entries[i] = entries[--size];
            } else {
                ++i;
            }
        }
    }

    template<typename Func>
    void each(Func func) const {
        for (unsigned i = 0; i < size; ++i) {
            func(entries[i].type_id);
        }
    }
};

}

#endif // EDYN_NETWORKING_COMP_NETWORK_DIRTY_HPP
