#ifndef CAN_INTERFACE_DRIVER_H
#define CAN_INTERFACE_DRIVER_H

#ifdef __cplusplus

extern "C" {

#endif

#include <stdbool.h>
#include <stdint.h>

#define CANFD_MTU 72
#define CAN_MAX_DLEN 8

typedef struct {
    uint32_t id;
    uint8_t length;
    uint8_t data[CANFD_MTU];
    bool is_extended;
    bool is_fd;
} CANFD_Message;

/*Enum containing the different CAN receive ID */
typedef enum {
    ENCODER_ANGLES = 0x47A,
    TEMP,
    PRESSURE,
} CAN_RECIEVE_MESSAGE_ID;

/*Enum contain the different CAN transmit ID */
typedef enum {
    STOP_THRUSTERS = 0x369,
    START_THRUSTERS,
    SET_THRUSTER_PWM,
    SET_LED_PWM,
    RESET_THRUSTER_MCU,
    STOP_GRIPPER = 0x469,
    START_GRIPPER,
    SET_GRIPPER_PWM,
    RESET_GRIPPER_MCU

} CAN_TRANSMIT_MESSAGE_ID;


/**
 * @brief Initializes CAN socket
 * @param CAN interface
 * @return -1 on failure and 0 on success
 */
int canfd_init(const char* interface);
/**
 * @brief send CAN frame
 * @param pointer to CAN message
 * @return -1 on failure and 0 on success
 */
int canfd_send(const CANFD_Message* msg);

/**
 * @brief recieves CAN frame
 * @param pointer to CAN message
 * @param timeout in ms
 * @return -1 on failure and 0 on success
 */
int canfd_recieve(CANFD_Message* msg, int timout_ms);

/**
  * @brief Closes CAN socket
  */
void canfd_close();

/**
  * @brief Sets CAN id filtering 
  * @param start_id
  * @param id mask
  */
void set_can_filter(uint16_t start_id, uint16_t id_mask);

#ifdef __cplusplus

}

#endif

#endif  // !CAN_INTERFACE_DRIVER_H
