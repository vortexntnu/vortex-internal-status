#include "driver/am_driver_single.hpp"

AcousticModemDriver::AcousticModemDriver(const std::string& device,
                                         int baudrate,
                                         int channel,
                                         int level,
                                         bool diagnostic,
                                         float timeout): 
                    AcousticModemDriverBase(timeout, channel, level, diagnostic),
                    io_(),
                    m_serial_port(io_),
                    device_(device)
                    {
    
    open(device,baudrate);
    set_channel(channel);
    set_level(level);
    
    std::cout << "[INFO] Modem initialized on device " << this->device_
              << ", channel " << channel << ", level " << level
              << ", diagnostic mode = " << std::boolalpha << diagnostic
              << std::endl;
}

void AcousticModemDriver::open(const std::string& device, int& baudrate){
    m_recv_buffer.resize(m_recv_buffer_size);
    m_serial_port.open(device);
    m_serial_port.set_option(asio::serial_port_base::baud_rate(baudrate));
    m_serial_port.set_option(asio::serial_port_base::flow_control(asio::serial_port_base::flow_control::none));
    m_serial_port.set_option(asio::serial_port_base::parity(asio::serial_port_base::parity::none));
    m_serial_port.set_option(asio::serial_port_base::stop_bits(asio::serial_port_base::stop_bits::one));

    start_async_read();
    io_thread_ = std::thread([this] { io_.run(); });
}

AcousticModemDriver::~AcousticModemDriver() {
    asio::error_code error;
    m_serial_port.close(error);
}


void AcousticModemDriver::start_async_read() {
    m_func=[this](std::vector<uint8_t>& buffer, const size_t& bytes_transferred) {
            std::vector<uint8_t> data(buffer.begin(),
                                      buffer.begin() + bytes_transferred);
            this->read_callback(data);
        };
    m_serial_port.async_read_some(
        asio::buffer(m_recv_buffer),
        [this](std::error_code error, size_t bytes_transferred)
        {
        async_receive_handler(error, bytes_transferred);
        });
    
}

void AcousticModemDriver::async_receive_handler(const asio::error_code & error,size_t bytes_transferred) {
    if (error) {
        this->close();
        return;
    }
    if (bytes_transferred > 0 && m_func) {
        m_func(m_recv_buffer, bytes_transferred);
        
    }
    m_serial_port.async_read_some(
            asio::buffer(m_recv_buffer),
            [this](std::error_code error, size_t bytes_transferred)
            {
                async_receive_handler(error, bytes_transferred);
            });
}

size_t AcousticModemDriver::write_raw(const uint8_t* data, size_t size){
    return m_serial_port.write_some(asio::buffer(data, size));
}
void AcousticModemDriver::close() {
    asio::error_code error;
    m_serial_port.close(error);
}