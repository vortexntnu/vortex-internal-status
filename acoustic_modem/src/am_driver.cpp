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
  // TODO: add verifications?
  port_->close();
}

int AcousticModemDriver::send_data(std::string data){
  // send data using serial driver's send(msg)
  std::vector<uint8_t> msg(data.begin(), data.end());
  int bytes=port_->send(msg);
  return bytes;
}

// overload send_data(char)
int AcousticModemDriver::send_data(char data){
  // send data using serial driver's send(msg)
  std::vector<uint8_t> msg = { static_cast<uint8_t>(data) };
  int bytes=port_->send(msg);
  return bytes;
}

int AcousticModemDriver::send_two_bytes(std::string data){
  // only send 2 bytes waiting 1 second after sending them
  if(data.length()!=2){
    return 0;
  }else{
    int bytes=this->send_data(data);

    // 10bps
    std::this_thread::sleep_for(std::chrono::seconds(2));
    return bytes;
  }
}

int AcousticModemDriver::send_msg(std::string data, float timeout){
  // TODO: implement in diagnostic mode, if needed

  int  sum_sent_char=0;

  if(data.length()%2 != 0){
    data+=' ';
  }
  for(int i=0; i<data.length();i+=2){
    std::string chunk=data.substr(i,2);
    int sent_chunk=this->send_two_bytes(chunk);
    if(sent_chunk!=0){
      sum_sent_char+=sent_chunk;
    }
    // wait for transmission
    std::this_thread::sleep_for(std::chrono::seconds(2));
  }
  return sum_sent_char;
}

// to set channel of communication
bool AcousticModemDriver::set_channel(int channel){
  // TODO: check that channel is a number between 1 and 12 and return false if not

  // wait 1 sec between c and c to go in command mode
  this->send_data('c');
  std::this_thread::sleep_for(std::chrono::seconds(1));
  this->send_data('c');

  switch (channel)
  {
  case 10:
    this->send_data('a');
    break;  
  case 11:
    this->send_data('b');
    break;
  case 12:
    this->send_data('c');
    break;
  default:
    this->send_data(std::to_string(channel));
    break;
  }
  channel_=channel;
  std::this_thread::sleep_for(std::chrono::seconds(1));
  return true;
}

// to set power level
bool AcousticModemDriver::set_level(int level){
  // TODO: check level and return False

  this->send_data('l');
  std::this_thread::sleep_for(std::chrono::seconds(1));
  this->send_data('l');

  this->send_data(std::to_string(level));
  level_=level;
  std::this_thread::sleep_for(std::chrono::seconds(1));
  return true;
}

// to set diagnostic mode?
bool AcousticModemDriver::set_diagnostic_mode(bool diagnostic){
  // TODO
  return false;
}

std::optional<DiagnosticData> decode_packet(const std::vector<uint8_t>& packet){
  std::string packet_str(packet.begin(), packet.end());

  if (packet_str.size()!=18 || packet_str.front()!='$' || packet_str.back()!='\n') {
    return std::nullopt; 
  }

  std::vector<uint8_t> data_bytes(packet.begin()+1, packet.begin()+17);

  // TODO
}

void AcousticModemDriver::close(){
  port_->close();
}  

