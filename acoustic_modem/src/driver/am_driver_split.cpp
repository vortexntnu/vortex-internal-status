#include "driver/am_driver_split.hpp"

AcousticModemDriverSplit::AcousticModemDriverSplit(const std::string& tx_device,
                                                   const std::string& rx_device,
                                                   int baudrate,
                                                   int channel,
                                                   int level,
                                                   bool diagnostic,
                                                   float timeout):
    AcousticModemDriverBase(channel, level, diagnostic, timeout), 
    io_(),
      tx_serial_port_(io_),
      rx_serial_port_(io_),
      tx_device_(tx_device),
      rx_device_(rx_device)
{
    open(tx_device, rx_device, baudrate);
    set_channel(channel);
    set_level(level);

    std::cout << "[INFO] Split modem initialized TX=" << tx_device_
              << " RX=" << rx_device_
              << ", channel " << channel
              << ", level " << level
              << ", diagnostic mode = " << std::boolalpha << diagnostic
              << std::endl;
}

void AcousticModemDriverSplit::open(const std::string& tx_device,
                                    const std::string& rx_device,
                                    int& baudrate)
{
    m_recv_buffer.resize(m_recv_buffer_size);

    tx_serial_port_.open(tx_device);
    rx_serial_port_.open(rx_device);

    tx_serial_port_.set_option(asio::serial_port_base::baud_rate(baudrate));
    tx_serial_port_.set_option(asio::serial_port_base::flow_control(asio::serial_port_base::flow_control::none));
    tx_serial_port_.set_option(asio::serial_port_base::parity(asio::serial_port_base::parity::none));
    tx_serial_port_.set_option(asio::serial_port_base::stop_bits(asio::serial_port_base::stop_bits::one));

    rx_serial_port_.set_option(asio::serial_port_base::baud_rate(baudrate));
    rx_serial_port_.set_option(asio::serial_port_base::flow_control(asio::serial_port_base::flow_control::none));
    rx_serial_port_.set_option(asio::serial_port_base::parity(asio::serial_port_base::parity::none));
    rx_serial_port_.set_option(asio::serial_port_base::stop_bits(asio::serial_port_base::stop_bits::one));

    start_async_read();
    io_thread_ = std::thread([this] { io_.run(); });
}

AcousticModemDriverSplit::~AcousticModemDriverSplit() {
    asio::error_code error;
    tx_serial_port_.close(error);
    rx_serial_port_.close(error);
}
void AcousticModemDriverSplit::close()
{
    asio::error_code error;
    tx_serial_port_.close(error);
    rx_serial_port_.close(error);
}

size_t AcousticModemDriverSplit::write_raw(const uint8_t* data, size_t size) {
    return tx_serial_port_.write_some(asio::buffer(data, size));
}

void AcousticModemDriverSplit::start_async_read() {
    m_func=[this](std::vector<uint8_t>& buffer, const size_t& bytes_transferred) {
            std::vector<uint8_t> data(buffer.begin(),
                                      buffer.begin() + bytes_transferred);
            this->read_callback(data);
        };
    rx_serial_port_.async_read_some(
        asio::buffer(m_recv_buffer),
        [this](std::error_code error, size_t bytes_transferred)
        {
        async_receive_handler(error, bytes_transferred);
        });
    
}

void AcousticModemDriverSplit::async_receive_handler(const asio::error_code & error,size_t bytes_transferred) {
    if (error) {
        this->close();
        return;
    }
    if (bytes_transferred > 0 && m_func) {
        m_func(m_recv_buffer, bytes_transferred);
        
    }
    rx_serial_port_.async_read_some(
            asio::buffer(m_recv_buffer),
            [this](std::error_code error, size_t bytes_transferred)
            {
                async_receive_handler(error, bytes_transferred);
            });
}


