#ifndef CAN_DECODE_HPP_

#include <string>
#include <cstdint>


#define CAN_DECODE_HPP_


#define NUM_ANGLES 2
#define CELLS_COUNT        6
#define CELLS_PAYLOAD_LEN  12
#define CAN_ID_BOTHOFF_CMD 0x200  //EXAMPLE VALUE
#define BOTHOFF_CMD_BYTE 0xA5
#define CAN_TEMP_ID 0x100         //EXAMPLE VALUE
// #define CAN_VOLTAGE_ID 0x101      //EXAMPLE VALUE

//legg til func for current, pressure, standbymode, reset mcu

#define CAN_PRESSURE_ID 0x102      //EXAMPLE VALUE
#define CAN_STANDBYMODE_ID 0x103     //EXAMPLE VALUE
#define CAN_RST_MCU 0x104     //EXAMPLE VALUE
// #define CAN_CURRENT_ID 0x105     //EXAMPLE VALUE

#define CAN_ALERT_SSA_ID   0x200u
#define CAN_ALERT_PFA_1_ID 0x201u
#define CAN_ALERT_PFA_2_ID 0x202u
#define CAN_CURRENT_ID     0x203u
#define CAN_VOLTAGE_ID 0x204      //EXAMPLE VALUE



std::string decode_encoder_angles(const uint8_t* data, size_t len);

std::string decode_motor_frames(const uint8_t* data, size_t len);

std::string decode_set_gripper_pwm(const uint8_t* data, size_t len);

std::string decode_gripper_start(const uint8_t* data, size_t len);

std::string decode_gripper_stop(const uint8_t* data, size_t len);

std::string decode_alert_ssa(const uint8_t* data, size_t len);

std::string decode_alert_pfa_1(const uint8_t* data, size_t len);

std::string decode_alert_pfa_2(const uint8_t* data, size_t len);

std::string decode_current(const uint8_t* data, size_t len);

std::string decode_temp(const uint8_t* data, size_t len);

std::string decode_voltage(const uint8_t* data, size_t len);

std::string decode_pressure_sample(const uint8_t* data, size_t len);

std::string decode_leakage_alarm(const uint8_t* data, size_t len);


#endif // !CAN_DECODE_HPP_
