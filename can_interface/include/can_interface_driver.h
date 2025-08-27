#ifndef CAN_INTERFACE_DRIVER_H
#define CAN_INTERFACE_DRIVER_H

#include <errno.h>
#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/bcm.h>
#include <linux/can/error.h>
#include <linux/can/gw.h>
#include <linux/can/isotp.h>
#include <linux/can/j1939.h>
#include <linux/can/netlink.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#ifdef __cplusplus
extern "C" {
#endif

/*Enum containing the different CAN receive ID */
typedef enum {
    ENCODER_ANGLES = 0x47A,
    TEMP_INTERNAL,
    PRESSURE_INTERNAL,
    PRESSURE_EXTERNAL,
    PSM
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
 * @param sock pointer sock value
 * @param interface char pointer to CAN interface
 * @return -1 on failure and 0 on success
 */
int canfd_init(int* sock, const char* interface);
/**
 * @brief send CAN frame
 * @param sock sock value
 * @param msg pointer to CAN message
 * @return -1 on failure and 0 on success
 */
int canfd_send(int sock, const struct canfd_frame* msg);

/**
 * @brief receives CAN frame
 * @param sock sock value
 * @param msg pointer to CAN message
 * @param timout_ms timeout in ms
 * @return -1 on failure
 *          0 on success
 */
int canfd_recieve(int sock, struct canfd_frame* msg, int timout_ms);

/**
 * @brief Closes CAN socket
 * @param sock pointer to sock variable
 */
void canfd_close(int* sock);

/**
 * @brief Sets CAN id filtering
 * @param start_id
 * @param id mask
 */
void set_can_filter(int sock, uint16_t start_id, uint16_t id_mask);

#ifdef __cplusplus
}
#endif

#endif  // !CAN_INTERFACE_DRIVER_H
