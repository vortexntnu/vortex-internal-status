#ifndef AM_DRIVER_HPP
#define AM_DRIVER_HPP

#include <chrono>
#include <fstream>  // per scrivere su file
#include <iostream>
#include <mutex>
#include <nlohmann/json.hpp>
#include <optional>
#include <queue>
#include <string>
#include <vector>
#include <asio.hpp>
#include <functional>

#include <io_context/io_context.hpp>
// #include <serial_driver/serial_driver.hpp>
// #include <serial_driver/serial_port.hpp>


/**
 * we don't need last, we know the dimension of the data we are sending
 * so we use the type to understand which send_data we use
*/
//  struct PacketHeader {
//     uint8_t type : 2;
//     uint8_t order : 3;
//     //uint8_t last : 1;
// };

enum class TxState{
    SENDING,
    WAIT_ACK,
    IDLE,
};


enum class MsgType : uint8_t{
    Type_def=0,
    Type_1 = 1,
    Type_2 = 2,
    Type_3 = 3,
    Type_4 = 4,
};

struct DiagnosticData {
    uint8_t TR_BLOCK[2];  // 6 bits will be used by the packet header
    uint8_t BER;
    uint8_t SIGNAL_POWER;
    uint8_t NOISE_POWER;
    uint8_t PACKET_VALID[2];
    uint8_t PACKET_INVALID;
    uint8_t GIT_REV;
    uint8_t TIME[3];
    uint8_t CHIP_ID[2];
    uint8_t HW_REV : 2;
    uint8_t CHANNEL : 4;
    uint8_t TB_VALID : 1;
    uint8_t TX_COMPLETE : 1;
    uint8_t DIAGNOSTIC_MODE : 1;
    uint8_t POWER_LEVEL : 2;
};

class AcousticModemDriver {
    //friend class TestAcousticModemDriver;
    public:
    //default for testing
    AcousticModemDriver();
    /**
        close the port
     */
    ~AcousticModemDriver();

    /**
        Initialize the modem connection.
        Parameters:
            device: (str): Serial port (e.g. "COM3" on Windows or "/dev/ttyUSB0"
    on Linux). baudrate (int): Baud rate (default 9600). timeout (float):
    Timeout for serial reads (default 0.5). channel (int): Channel to set (valid
    values 1 to 12), (default 1). level (int): Power level to set (valid values
    1 to 4), (default 4). diagnostic (bool): If True, set the modem to
    diagnostic mode; if False, set transparent mode, (default 1).
    **/
    AcousticModemDriver(const std::string& device,
                        int baudrate,
                        int channel,
                        int level,
                        bool diagnostic,
                        float timeout);

    std::string make_handshake(MsgType type);

    // receiver will use these functions to implement the logic 
    bool is_handshake(uint16_t packet);
    MsgType type(uint16_t packet);
    uint16_t id(uint16_t packet);
    
    // distinguish each type of messsage and associate a number of float with it
    static size_t floats_for_type(MsgType t);

    // divide the float in 2 uint16
    static void float_to_word(float v, std::string &s0,std::string &s1);

    static float word_to_float(uint16_t w0, uint16_t w1);

    // convert float to string and use send_two_bytes
    void send_word(uint16_t w);

    // send whole messagge with handshake and chunks
    void send_message(MsgType type, const float* data);


    void rx_start_handshake(uint16_t hs_word);
    
    void rx_reset();

    bool rx_rebuild_word(uint16_t w, MsgType &out_type, uint16_t &out_msg_id, float *out_floats, uint8_t &inout_capacity, std::chrono::milliseconds timeout);

    bool try_pop_rx(std::vector<uint8_t> &out_w);
    /**
        Send ASCII data to the modem.

        Parameters:
            data (str): The data to be sent.

        Returns:
            int: Number of characters written.
    **/
    size_t send_data(std::string data);

    /**
        Send ASCII data to the modem.

        Parameters:
            data (char): The data to be sent.

        Returns:
            int: Number of characters written.
    **/
    size_t send_data(char data);

    /**
        Send two bytes of data to the modem.

        Parameters:
            data (str): A string (at least two characters) representing the
    data.

        Returns:
            int: Number of characters written
    **/
    size_t send_two_bytes(std::string data);

    /**
        Send a longer message (more than 2 bytes) in 2-byte chunks.
        If in diagnostic mode, after sending each chunk, wait for a diagnostic
       report that indicates the transmission is complete (TX_COMPLETE == 1). If
       in transparent mode, simply wait 2 seconds between chunks.

        Parameters:
            data (str): The message to be sent.
            timeout(float): Maximum time (in seconds) to wait for TX_COMPLETE
            after sending each 2-byte chunk.
    */
    size_t send_msg(std::string data, float timeout = 5.0f);
    
    /**
        Read data from the serial port and search for a valid diagnostic packet.
        A valid packet starts with '$' (0x24) and ends with '\\n' (0x0A) and is
       exactly 18 bytes long.

        Returns:
            Optional[bytes]: The valid packet if found, otherwise the buffer if
       it is not empty.
     */

    // to set channel of communication
    bool set_channel(int channel);

    // to set power level
    bool set_level(int level);

    // to set diagnostic mode
    bool set_diagnostic_mode();

    bool reset_diagnostic_mode();

    /**
        Request a diagnostic report, decode it, update member variables from the
       report,

        TODO: and optionally save the report as a JSON file.

        This function sends the report request command and then listens for a
       valid packet until overall_timeout seconds have elapsed.

        Parameters:
            TODO: filename (str, optional): If provided, the report is saved to
       this file. overall_timeout (float): Maximum time (in seconds) to wait for
       a valid report.

        Returns:
            DiagnosticData: The decoded report if successful; otherwise, None.
     */
    std::optional<DiagnosticData> request_report(
        float overall_timeout = 5.0f,
        std::optional<std::string> filename = std::nullopt);

    /**
        Update internal state modem configuration
        Parameters:
            DiagnosticData report: Decoded report from the modem containing
       configuration info.
     */
    void update_state_from_report(DiagnosticData report);

    /**
        Request a diagnostic report from the modem.
    */
    void get_report();

    /**
       Read data from the serial port and search for a valid diagnostic packet.
        A valid packet starts with '$' (0x24) and ends with '\\n' (0x0A) and is
       exactly 18 bytes long.

        Returns:
            Optional[bytes]: The valid packet if found, otherwise the buffer if
       it is not empty.
     */

    std::optional<std::vector<uint8_t>> read_packet();

    /**
     * read Data asynchronously
     */
    void start_async_read();

    /**
     * callback for async_receive(), saves data in queue
     */
    void read_callback(std::vector<uint8_t>& data);

    /**
     * ideally in this we should order the different packet and decode the
     * message then it will be used in the ros2 node, how to decide which type
     * of msg? header?
     */
    std::optional<std::vector<uint8_t>> process_packet();

    /**
        Decode a diagnostic packet received from the modem.

        The packet should be 18 bytes long, starting with '$' (0x24) and ending
       with '\\n' (0x0A). The bytes between contain the data in the following
       format:
            - Byte 0: '$'
            - Bytes 1-16: Data fields (see modem documentation)
            - Byte 17: '\\n'

        Returns:
            A DiagnosticData of decoded values if the packet is valid,
            otherwise None.
     */

    std::optional<DiagnosticData> decode_packet(std::vector<uint8_t>& packet);

    /**
        Close the serial connection.
    */
    void close();

   private:
    
    void async_receive(std::function<void (std::vector<uint8_t> &, const size_t &)> func);
    void async_receive_handler(const asio::error_code & error,size_t bytes_transferred);
    asio::io_context io_;
    asio::serial_port m_serial_port;
    void open(const std::string& device, int& baudrate);
    size_t send(const std::vector<uint8_t> & buff);
    std::function<void (std::vector<uint8_t> &, const size_t &)> m_func;
    std::thread io_thread_;

    static constexpr size_t m_recv_buffer_size{2048};
    std::vector<uint8_t> m_recv_buffer;
   // TODO: could be done by using class BitWriter
    void append_bits(std::vector<uint8_t>& buffer,
                     uint16_t bit_to_append,
                     int count,
                     int& bit_position);

    struct DiagnosticPacket {
        uint16_t TR_BLOCK;
        uint8_t BER;
        uint8_t SIGNAL_POWER;
        uint8_t NOISE_POWER;
        uint16_t PACKET_VALID;
        uint8_t PACKET_INVALID;
        uint8_t GIT_REV;
        uint8_t TIME_L;
        uint8_t TIME_M;
        uint8_t TIME_H;
        uint16_t CHIP_ID;
        uint8_t HW_CH_FLAGS;       // 14  (contains HW_REV, CHANNEL, TB_VALID,
                                   // TX_COMPLETE)
        uint8_t MODE_LEVEL_FLAGS;  // 15 (contains DIAGNOSTIC_MODE, LEVEL)
    };

    // function called to get the port returns shared pointer: drv_.port()
    // drivers::serial_driver::SerialDriver drv_;
    // drivers::serial_driver::SerialPortConfig cfg_;
    // std::shared_ptr<drivers::serial_driver::SerialPort> port_;
    std::string device_;


    // 4 bits to recognize handshake
    static constexpr uint8_t HANDSHAKE_SYNC = 0xA;
    // 10 bit for msg id, to avoid error caused by lag or delay (TODO: to check if it is necessary)
    uint16_t msg_id;

    static constexpr uint16_t RX_MAX_WORDS = 20;
    //static constexpr uint8_t RX_MAX_FLOATS = 10;
    
    struct RxState{
        bool rx_receiving= false;
        MsgType rx_type=MsgType::Type_def;
        uint16_t rx_msg_id= 0;
        uint16_t rx_expected_words = 0;
        uint16_t rx_received_words = 0;
        uint16_t rx_words[RX_MAX_WORDS]{};
        uint8_t rx_n_floats= 0;
        std::chrono::steady_clock::time_point rx_last_rx{};
    };

    RxState rx;

    // to save what we receive asynchronously
    /**
     * needed because of async_read_some inserial lib
     * it could be called when reading data and when writing
     * so race dondition
     */
    std::queue<std::vector<uint8_t>> queue;
    std::map<uint8_t, std::map<uint8_t, uint16_t>> map;
    std::mutex queue_mutex;

    //std::queue<std::string> pending_msg;
    // keeps count on what bit are we are at when we recreate the message
    int bit_pos_;
    std::vector<uint8_t> full_message;

    int channel_;
    int level_;
    bool diagnostic_;

    // Timeout per chunk,
    // static constexpr float default_timeout = 0.5f;
};

#endif
