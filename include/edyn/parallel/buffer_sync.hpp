#ifndef EDYN_PARALLEL_BUFFER_SYNC_HPP
#define EDYN_PARALLEL_BUFFER_SYNC_HPP

#include <mutex>
#include <vector>
#include <memory>
#include <cstdint>

namespace edyn {

struct buffer_sync_context {
    using data_type = std::vector<uint8_t>;
    data_type data;
    std::mutex mutex;
};

class buffer_sync_reader;
class buffer_sync_writer;
using buffer_sync_reader_writer_pair = std::pair<buffer_sync_reader, buffer_sync_writer>;

class buffer_sync_writer {
public:
    buffer_sync_writer(const buffer_sync_writer &) = default;

    void write(const buffer_sync_context::data_type &data) {
        std::lock_guard lock(m_ctx->mutex);
        m_ctx->data = data;
    }
    
    friend buffer_sync_reader_writer_pair make_buffer_sync();

private:
    buffer_sync_writer(std::shared_ptr<buffer_sync_context> ctx)
        : m_ctx(ctx)
    {}

    std::shared_ptr<buffer_sync_context> m_ctx;
};

class buffer_sync_reader {
public:
    buffer_sync_reader(const buffer_sync_reader &) = default;

    void read(buffer_sync_context::data_type &data) {
        std::lock_guard lock(m_ctx->mutex);
        data = m_ctx->data;
    }

    friend buffer_sync_reader_writer_pair make_buffer_sync();

private:
    buffer_sync_reader(std::shared_ptr<buffer_sync_context> ctx)
        : m_ctx(ctx)
    {}

    std::shared_ptr<buffer_sync_context> m_ctx;
};

inline
buffer_sync_reader_writer_pair make_buffer_sync() {
    auto ctx = std::make_shared<buffer_sync_context>();
    return std::make_pair(buffer_sync_reader(ctx), buffer_sync_writer(ctx));
}

}

#endif // EDYN_PARALLEL_BUFFER_SYNC_HPP