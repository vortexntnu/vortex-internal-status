#include "am_driver.hpp"
#include <utility>


AcousticModemDriver::AcousticModemDriver(std::string& device,int baudrate,int channel,int level,bool diagnostic,float timeout)
    : io_(1),       // I/O context, required by the library
      drv_(io_),    // SerialDriver object
      cfg_(baudrate,    // configuration how UART should operate
      drivers::serial_driver::FlowControl::NONE,
      drivers::serial_driver::Parity::NONE,
      drivers::serial_driver::StopBits::ONE),
      // saves the device path to a member variable
      device_(device),
      channel_(channel),
      level_(level)   
      {
        // TODO: missing some verifications (port is really open, if device exists...)

        // initialization of the port in device_
        drv_.init_port(device_, cfg_);

        //creation of the acutal SerialPort object (create to use open(), close() ...)
        port_ = drv_.port();

        // actually open physical serial port
        port_->open();

        this->set_channel(channel);
        this->set_level(level);

        // TODO: diagnostic mode if needed
              

        std::cout << "[INFO] Modem initialized on device " << this->device_
              << ", channel " << channel
              << ", level " << level
              << ", diagnostic mode = " << std::boolalpha << diagnostic
              << std::endl;
}


AcousticModemDriver::~AcousticModemDriver(){
  // TODO
  return;
}

int AcousticModemDriver::send_two_bytes(std::string data){
  // TODO
  return 0;
}

int AcousticModemDriver::send_msg(std::string data, float timeout){
  // TODO
  return 0;
}

// to set channel of communication
bool AcousticModemDriver::set_channel(int channel){
  // TODO
  return false;
}

// to set power level
bool AcousticModemDriver::set_level(int level){
  // TODO
  return false;
}

// to set diagnostic mode
bool AcousticModemDriver::set_diagnostic_mode(bool diagnostic){
  // TODO
  return false;
}

void AcousticModemDriver::close(){
  // TODO
  return;
}  

