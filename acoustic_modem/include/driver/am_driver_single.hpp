#ifndef AM_DRIVER_HPP
#define AM_DRIVER_HPP

#include "driver/am_driver_base.hpp"

class AcousticModemDriver : public AcousticModemDriverBase {
    public:

    ~AcousticModemDriver();

    AcousticModemDriver(const std::string& device, int baudrate, int channel, int level, bool diagnostic, float timeout);
    void start_async_read();
    void async_receive_handler(const asio::error_code & error,size_t bytes_transferred);
    void open(const std::string& device, int& baudrate);
    void close();
    
   private:
    size_t write_raw(const uint8_t* data, size_t size) override;
    std::function<void (std::vector<uint8_t> &, const size_t &)> m_func;
    std::thread io_thread_;
    
    static constexpr size_t m_recv_buffer_size{2048};
    std::vector<uint8_t> m_recv_buffer;

    asio::io_context io_;
    asio::serial_port m_serial_port;
    std::string device_;
};

#endif
