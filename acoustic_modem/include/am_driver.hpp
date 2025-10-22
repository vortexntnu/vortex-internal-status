#ifndef AM_DRIVER_HPP
#define AM_DRIVER_HPP

#include<string>
#include<iostream>
#include<chrono>
#include<vector>
#include <io_context/io_context.hpp>
#include <serial_driver/serial_driver.hpp>
#include <serial_driver/serial_port.hpp>


struct PacketHeader{
    uint8_t type : 2;
    uint8_t order : 3;
    uint8_t last : 1;
};

struct DiagnosticData {
    uint8_t TR_BLOCK[2];
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
    uint8_t TX_COMPLETE :1;
    uint8_t DIAGNOSTIC_MODE : 1;
    uint8_t LEVEL : 2;
};

class AcousticModemDriver{
    /**
        close the port
     */
    ~AcousticModemDriver();
    
    /**  
        Initialize the modem connection.
        Parameters:
            device: (str): Serial port (e.g. "COM3" on Windows or "/dev/ttyUSB0" on Linux).
            baudrate (int): Baud rate (default 9600).
            timeout (float): Timeout for serial reads (default 0.5).
            channel (int): Channel to set (valid values 1 to 12), (default 1).
            level (int): Power level to set (valid values 1 to 4), (default 4).
            diagnostic (bool): If True, set the modem to diagnostic mode; if False, set transparent mode, (default 1).
    **/
    AcousticModemDriver(std::string& device,int baudrate=9600,int channel=1,int level=4,bool diagnostic=false,float timeout=default_timeout);



    /**
        Send ASCII data to the modem.
        
        Parameters:
            data (str): The data to be sent.

        Returns: 
            int: Number of characters written.
    **/
    int send_data(std::string data);

/**
        Send ASCII data to the modem.
        
        Parameters:
            data (char): The data to be sent.

        Returns: 
            int: Number of characters written.
    **/
    int send_data(char data);


    /**
        Send two bytes of data to the modem.
        
        Parameters:
            data (str): A string (at least two characters) representing the data.

        Returns:
            int: Number of characters written
    **/
    int send_two_bytes(std::string data);

    /**
        Send a longer message (more than 2 bytes) in 2-byte chunks.
        If in diagnostic mode, after sending each chunk, wait for a diagnostic report 
        that indicates the transmission is complete (TX_COMPLETE == 1).
        If in transparent mode, simply wait 2 seconds between chunks.
        
        Parameters:
            data (str): The message to be sent.
            timeout(float): Maximum time (in seconds) to wait for TX_COMPLETE
            after sending each 2-byte chunk.
    */
    int send_msg(std::string data, float timeout=default_timeout);


    /**
    Read data from the serial port and search for a valid diagnostic packet.
    A valid packet starts with '$' (0x24) and ends with '\\n' (0x0A) and is exactly 18 bytes long.
    
    Returns:
        Optional[bytes]: The valid packet if found, otherwise the buffer if it is not empty.
     */

    

    // to set channel of communication
    bool set_channel(int channel);

    // to set power level
    bool set_level(int level);

    // to set diagnostic mode
    bool set_diagnostic_mode(bool diagnostic);

    /**
     * Read data from the serial port and search for a valid diagnostic packet.
        A valid packet starts with '$' (0x24) and ends with '\\n' (0x0A) and is exactly 18 bytes long.
        
        Returns:
            Optional[bytes]: The valid packet if found, otherwise the buffer if it is not empty.
     */

    std::vector<uint8_t> read_packet();

    /**
    Decode a diagnostic packet received from the modem.
        
    The packet should be 18 bytes long, starting with '$' (0x24) and ending with '\\n' (0x0A).
    The bytes between contain the data in the following format:
        - Byte 0: '$'
        - Bytes 1-16: Data fields (see modem documentation)
        - Byte 17: '\\n'
    
    Returns:
        TODO:Optional[Dict[str, Any]]: A dictionary of decoded values if the packet is valid,
        otherwise None.
     */

    //std::optional<DiagnosticData> decode_packet(std::vector<uint8_t>& packet);

    /**
        Close the serial connection.
    */
    void close();   

    private:

    drivers::common::IoContext io_;

    // function called to get the port returns shared pointer: drv_.port()
    drivers::serial_driver::SerialDriver drv_;
    drivers::serial_driver::SerialPortConfig cfg_;
    std::shared_ptr<drivers::serial_driver::SerialPort> port_;
    std::string device_;
    int channel_;
    int level_;

    // Timeout per chunk,
    static constexpr float default_timeout = 0.5f;


};

#endif