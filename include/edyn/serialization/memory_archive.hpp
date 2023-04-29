#ifndef EDYN_SERIALIZATION_MEMORY_ARCHIVE_HPP
#define EDYN_SERIALIZATION_MEMORY_ARCHIVE_HPP

#include <cstdint>
#include <cstddef>
#include <type_traits>
#include <vector>
#include <array>
#include <map>
#include "edyn/util/tuple_util.hpp"
#include "edyn/serialization/s11n_util.hpp"

namespace edyn {

class memory_input_archive {
public:
    using data_type = uint8_t;
    using buffer_type = const data_type*;
    using is_input = std::true_type;
    using is_output = std::false_type;

    memory_input_archive(buffer_type buffer, size_t size)
        : m_buffer(buffer)
        , m_size(size)
        , m_position(0)
        , m_failed(false)
    {}

    template<typename T>
    void operator()(T& t) {
        if constexpr(std::is_fundamental_v<T>) {
            read_bytes(t);
        } else if constexpr(!std::is_empty_v<T>) {
            serialize(*this, t);
        }
    }

    template<typename... Ts>
    void operator()(Ts&... t) {
        (operator()(t), ...);
    }

    bool failed() const {
        return m_failed;
    }

    bool eof() const {
        return m_position == m_size;
    }

protected:
    template<typename T>
    void read_bytes(T &t) {
        if (m_failed) return;

        if (m_position + sizeof(T) > m_size) {
            m_failed = true;
            return;
        }

        auto* buff = reinterpret_cast<const T*>(m_buffer + m_position);
        t = *buff;
        m_position += sizeof(T);
    }

protected:
    buffer_type m_buffer;
    const size_t m_size;
    size_t m_position;
    bool m_failed;
};

class memory_output_archive {
public:
    using data_type = uint8_t;
    using buffer_type = std::vector<data_type>;
    using is_input = std::false_type;
    using is_output = std::true_type;

    memory_output_archive(buffer_type& buffer)
        : m_buffer(&buffer)
    {}

    template<typename T>
    void operator()(T& t) {
        if constexpr(std::is_fundamental_v<T>) {
            write_bytes(t);
        } else if constexpr(!std::is_empty_v<T>) {
            serialize(*this, t);
        }
    }

    template<typename T>
    void operator()(const T& t) {
        if constexpr(std::is_fundamental_v<T>) {
            write_bytes(t);
        } else if constexpr(!std::is_empty_v<T>) {
            auto &ct = const_cast<T &>(t);
            serialize(*this, ct);
        }
    }

    template<typename... Ts>
    void operator()(Ts&... t) {
        (operator()(t), ...);
    }

protected:
    template<typename T>
    void write_bytes(const T &t) {
        auto idx = m_buffer->size();
        m_buffer->resize(idx + sizeof(T));
        auto *dest = reinterpret_cast<T*>(&(*m_buffer)[idx]);
        *dest = t;
    }

    buffer_type *m_buffer;
};

class fixed_memory_output_archive {
public:
    using data_type = uint8_t;
    using buffer_type = data_type*;
    using is_input = std::false_type;
    using is_output = std::true_type;

    fixed_memory_output_archive(buffer_type buffer, size_t size)
        : m_buffer(buffer)
        , m_size(size)
        , m_position(0)
        , m_failed(false)
    {}

    template<typename T>
    void operator()(T& t) {
        if constexpr(std::is_fundamental_v<T>) {
            write_bytes(t);
        } else if constexpr(!std::is_empty_v<T>) {
            serialize(*this, t);
        }
    }

    template<typename... Ts>
    void operator()(Ts&... t) {
        (operator()(t), ...);
    }

    bool failed() const {
        return m_failed;
    }

protected:
    template<typename T>
    void write_bytes(T &t) {
        if (m_failed) return;

        if (m_position + sizeof(T) > m_size) {
            m_failed = true;
            return;
        }

        auto *dest = reinterpret_cast<T*>(m_buffer + m_position);
        *dest = t;
        m_position += sizeof(T);
    }

    buffer_type m_buffer;
    size_t m_size;
    size_t m_position;
    bool m_failed;
};

}

#endif // EDYN_SERIALIZATION_MEMORY_ARCHIVE_HPP
