#ifndef AM_DRIVER_HPP
#define AM_DRIVER_HPP

#include <serial_driver/serial_driver.hpp>
#include <serial_driver/serial_port.hpp>



class AcousticModemDriver{
    /**
        close the port
     */
    ~AcousticModemDriver();
    
    /**  
        Initialize the modem connection.
        Parameters:
            port (str): Serial port (e.g. "COM3" on Windows or "/dev/ttyUSB0" on Linux).
            baudrate (int): Baud rate (default 9600).
            timeout (float): Timeout for serial reads (default 0.5).
            channel (int): Channel to set (valid values 1 to 12), (default 1).
            level (int): Power level to set (valid values 1 to 4), (default 4).
            diagnostic (bool): If True, set the modem to diagnostic mode; if False, set transparent mode, (default 1).
    **/
    AcousticModemDriver(string port,int baudrate=9600,int channel=1,int level=4,bool diagnostic=False,float timeout=default_timeout);


    /**
        Send two bytes of data to the modem.
        
        Parameters:
            data (str): A string (at least two characters) representing the data.

        Returns:
            int: Number of characters written
    **/
    int send_two_bytes(string data);

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
    int send_msg(string data, float timeout=default_timeout);

    /**
        Close the serial connection.
    */


    // to set channel of communication
    bool set_channel(int channel);

    void close();   

    private:

    string port;
    int channel;
    int level;

    // Timeout per chunk,
    static constexpr float default_timeout = 0.5f;


}

#endif