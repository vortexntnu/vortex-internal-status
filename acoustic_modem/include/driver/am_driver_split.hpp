#ifndef AM_DRIVER_SPLIT_HPP
#define AM_DRIVER_SPLIT_HPP

#include "driver/am_driver_base.hpp"

class AcousticModemDriverSplit : public AcousticModemDriverBase {
public:
    AcousticModemDriverSplit(const std::string& tx_device,
                             const std::string& rx_device,
                             int baudrate,
                             int channel,
                             int level,
                             bool diagnostic,
                             float timeout);

    ~AcousticModemDriverSplit();

    void open(const std::string& tx_device,
              const std::string& rx_device,
              int& baudrate);

    void close();

    void start_async_read();
    void async_receive_handler(const asio::error_code& error,
                               size_t bytes_transferred);

private:
    size_t write_raw(const uint8_t* data, size_t size) override;
    asio::io_context io_;
    asio::serial_port tx_serial_port_;
    asio::serial_port rx_serial_port_;
    std::thread io_thread_;

    std::vector<uint8_t> m_recv_buffer;
    static constexpr size_t m_recv_buffer_size{2048};

    std::function<void(std::vector<uint8_t>&, const size_t&)> m_func;

    std::string tx_device_;
    std::string rx_device_;

};

#endif