#ifndef EDYN_SERIALIZATION_FILE_ARCHIVE_HPP
#define EDYN_SERIALIZATION_FILE_ARCHIVE_HPP

#include <fstream>
#include <cstdint>
#include <type_traits>

#include "edyn/config/config.h"
#include "math_s11n.hpp"
#include "std_s11n.hpp"
#include "static_tree_s11n.hpp"
#include "triangle_mesh_s11n.hpp"

namespace edyn {

template<typename Archive, typename T>
void serialize(Archive &, T &);

class file_input_archive {
public:
    using is_input = std::true_type;
    using is_output = std::false_type;

    file_input_archive(const std::string &path)
        : m_file(path, std::ios::binary | std::ios::in)
    {
    }

    bool is_file_open() const {
        return m_file.is_open();
    }

    template<typename... Ts>
    void operator()(Ts&&... t) {
        if constexpr(sizeof...(Ts) == 1) {
            (serialize(*this, t), ...);
        }
        else {
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
        EDYN_ASSERT(m_file.is_open() && !m_file.eof());
        m_file.read(reinterpret_cast<char *>(&t), sizeof t);
    }

private:
    std::ifstream m_file;
};

class file_output_archive {
public:
    using is_input = std::false_type;
    using is_output = std::true_type;

    file_output_archive(const std::string &path)
        : m_file(path, std::ios::binary | std::ios::out)
    {
        EDYN_ASSERT(m_file.good());
    }

    template<typename... Ts>
    void operator()(Ts&&... t) {
        if constexpr(sizeof...(Ts) == 1) {
            (serialize(*this, const_cast<std::add_lvalue_reference_t<std::remove_const_t<std::remove_reference_t<Ts>>>>(t)), ...);
        }
        else {
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
        m_file.write(reinterpret_cast<char *>(&t), sizeof t);
    }

private:
    std::ofstream m_file;
};

class file_input_archive_source {
public:
    file_input_archive_source(const std::string &base_path)
        : m_base_path(base_path)
    {}

    file_input_archive operator()(size_t idx) {
        auto path = m_base_path + std::to_string(idx);
        auto input = file_input_archive(path);
        EDYN_ASSERT(input.is_file_open());
        return input;
    }

private:
    std::string m_base_path;
};

class file_output_archive_source {
public:
    file_output_archive_source(const std::string &base_path)
        : m_base_path(base_path)
    {}

    file_output_archive operator()(size_t idx) {
        auto path = m_base_path + std::to_string(idx);
        auto output = file_output_archive(path);
        return output;
    }

private:
    std::string m_base_path;
};

}

#endif // EDYN_SERIALIZATION_FILE_ARCHIVE_HPP