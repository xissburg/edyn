#ifndef EDYN_SERIALIZATION_MEMORY_ARCHIVE_HPP
#define EDYN_SERIALIZATION_MEMORY_ARCHIVE_HPP

#include <cstdint>
#include <type_traits>
#include <vector>
#include <map>

namespace edyn {

class memory_input_archive {
public:
    using data_type = uint8_t;
    using buffer_type = std::vector<data_type>;
    using is_input = std::true_type;
    using is_output = std::false_type;

    memory_input_archive(buffer_type &buffer)
        : m_buffer(&buffer)
        , m_offset(0)
    {}

    template<typename... Ts>
    void operator()(Ts&&... t) {
        if constexpr(sizeof...(Ts) == 1) {
            (serialize(*this, t), ...);
        } else {
            (operator()(t), ...);
        }
    }

    void operator()(char &t) {
        read_bytes(t);
    }

    void operator()(bool &t) {
        read_bytes(t);
    }

    void operator()(int8_t &t) {
        read_bytes(t);
    }

    void operator()(uint8_t &t) {
        read_bytes(t);
    }

    void operator()(int16_t &t) {
        read_bytes(t);
    }

    void operator()(uint16_t &t) {
        read_bytes(t);
    }

    void operator()(int32_t &t) {
        read_bytes(t);
    }

    void operator()(uint32_t &t) {
        read_bytes(t);
    }

    void operator()(int64_t &t) {
        read_bytes(t);
    }

    void operator()(uint64_t &t) {
        read_bytes(t);
    }

    void operator()(float &t) {
        read_bytes(t);
    }

    void operator()(double &t) {
        read_bytes(t);
    }

    template<typename T>
    void read_bytes(T &t) {
        auto* buff = reinterpret_cast<const T*>(&(*m_buffer)[m_offset]);
        t = *buff;
        m_offset += sizeof(T);
    }

private:
    buffer_type *m_buffer;
    size_t m_offset;
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

    template<typename... Ts>
    void operator()(Ts&&... t) {
        if constexpr(sizeof...(Ts) == 1) {
            (serialize(*this, const_cast<std::add_lvalue_reference_t<std::remove_const_t<std::remove_reference_t<Ts>>>>(t)), ...);
        } else {
            (operator()(t), ...);
        }
    }
    
    void operator()(char &t) {
        write_bytes(t);
    }

    void operator()(bool &t) {
        write_bytes(t);
    }

    void operator()(int8_t &t) {
        write_bytes(t);
    }

    void operator()(uint8_t &t) {
        write_bytes(t);
    }

    void operator()(int16_t &t) {
        write_bytes(t);
    }

    void operator()(uint16_t &t) {
        write_bytes(t);
    }

    void operator()(int32_t &t) {
        write_bytes(t);
    }

    void operator()(uint32_t &t) {
        write_bytes(t);
    }

    void operator()(int64_t &t) {
        write_bytes(t);
    }

    void operator()(uint64_t &t) {
        write_bytes(t);
    }

    void operator()(float &t) {
        write_bytes(t);
    }

    void operator()(double &t) {
        write_bytes(t);
    }

    template<typename T>
    void write_bytes(T &t) { 
        auto idx = m_buffer->size();
        m_buffer->resize(idx + sizeof(T));
        auto *dest = reinterpret_cast<T*>(&(*m_buffer)[idx]);
        *dest = t;
    }

private:
    buffer_type *m_buffer;
};

class memory_input_archive_source {
public:
    using buffer_type = std::map<size_t, memory_input_archive::buffer_type>;

    memory_input_archive_source(buffer_type &buffer)
        : m_buffer(&buffer)
    {}

    memory_input_archive operator()(size_t idx) {
        auto input = memory_input_archive((*m_buffer)[idx]);
        return input;
    }

private:
    buffer_type *m_buffer;
};

class memory_output_archive_source {
public:
    using buffer_type = std::map<size_t, memory_output_archive::buffer_type>;

    memory_output_archive_source(buffer_type &buffer) 
        : m_buffer(&buffer)
    {}

    memory_output_archive operator()(size_t idx) {
        auto output = memory_output_archive((*m_buffer)[idx]);
        return output;
    }

private:
    buffer_type *m_buffer;
};

}

#endif // EDYN_SERIALIZATION_MEMORY_ARCHIVE_HPP