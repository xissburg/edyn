#include "../common/common.hpp"

class message_queue_test: public ::testing::Test {
public:
    void on_int(int i) {
        m_value = i;
    }

    int m_value;

    std::unique_ptr<edyn::message_queue_input> m_input;
    std::unique_ptr<edyn::message_queue_output> m_output;

protected:
    void SetUp() override {
        m_value = 0;
        auto [msgq_in, msgq_out] = edyn::make_message_queue_input_output();
        m_input = std::make_unique<edyn::message_queue_input>(msgq_in);
        m_output = std::make_unique<edyn::message_queue_output>(msgq_out);
        m_output->sink<int>().connect<&message_queue_test::on_int>(*this);
    }

    void TearDown() override {

    }
};

TEST_F(message_queue_test, write_read_messages) {
    m_input->send<int>(667);
    ASSERT_NE(m_value, 667);
    m_output->update();
    ASSERT_EQ(m_value, 667);
}