#ifndef EDYN_SERIALIZATION_FILE_ARCHIVE_HPP
#define EDYN_SERIALIZATION_FILE_ARCHIVE_HPP

#include <fstream>
#include <type_traits>

#include "edyn/serialization/s11n_util.hpp"
#include "edyn/config/config.h"

namespace edyn {

class file_input_archive {
public:
    using is_input = std::true_type;
    using is_output = std::false_type;

    file_input_archive() {}

    file_input_archive(const std::string &path)
        : m_file(path, std::ios::binary | std::ios::in)
    {}

    void open(const std::string &path) {
        m_file.open(path, std::ios::binary | std::ios::in);
    }

    bool is_file_open() const {
        return m_file.is_open();
    }

    bool is_file_at_end() const {
        return m_file.eof();
    }

    template<typename... Ts>
    void operator()(Ts&&... t) {
        if constexpr(sizeof...(Ts) == 1) {
            (serialize(*this, t), ...);
        } else {
            (operator()(t), ...);
        }
    }

    void operator()(bool &t) {
        read_bytes(t);
    }

    void operator()(char &t) {
        read_bytes(t);
    }

    void operator()(unsigned char &t) {
        read_bytes(t);
    }

    void operator()(short &t) {
        read_bytes(t);
    }

    void operator()(unsigned short &t) {
        read_bytes(t);
    }

    void operator()(int &t) {
        read_bytes(t);
    }

    void operator()(unsigned int &t) {
        read_bytes(t);
    }

    void operator()(long &t) {
        read_bytes(t);
    }

    void operator()(unsigned long &t) {
        read_bytes(t);
    }

    void operator()(long long &t) {
        read_bytes(t);
    }

    void operator()(unsigned long long &t) {
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

    void seek_position(size_t pos) {
        m_file.seekg(pos);
    }

    size_t tell_position() {
        return m_file.tellg();
    }

protected:
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
        } else {
            (operator()(t), ...);
        }
    }

    void operator()(bool &t) {
        write_bytes(t);
    }

    void operator()(char &t) {
        write_bytes(t);
    }

    void operator()(unsigned char &t) {
        write_bytes(t);
    }

    void operator()(short &t) {
        write_bytes(t);
    }

    void operator()(unsigned short &t) {
        write_bytes(t);
    }

    void operator()(int &t) {
        write_bytes(t);
    }

    void operator()(unsigned int &t) {
        write_bytes(t);
    }

    void operator()(long &t) {
        write_bytes(t);
    }

    void operator()(unsigned long &t) {
        write_bytes(t);
    }

    void operator()(long long &t) {
        write_bytes(t);
    }

    void operator()(unsigned long long &t) {
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

    void close() {
        m_file.close();
    }

private:
    std::ofstream m_file;
};

}

#endif // EDYN_SERIALIZATION_FILE_ARCHIVE_HPP