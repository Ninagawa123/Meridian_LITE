#ifndef __MERIDIAN_LOCAL_FUNC__
#define __MERIDIAN_LOCAL_FUNC__

#include <cstdint>
#include <string>

/**
 * @brief Initialize wifi.
 *
 * @param[in] const char wifi_ap_ssid.
 * @param[in] const char wifi_ap_pass.
 */
void init_wifi(const char *wifi_ap_ssid, const char *wifi_ap_pass);

/**
 * @brief Receive meridim data from UDP.
 *        Received data keep on r_udp_meridim.
 *
 */
void receiveUDP();

/**
 * @brief Send meridim data to UDP.
 *
 */
void sendUDP();

/**
 * @brief Check SD card read and write.
 *
 */
void check_sd();

/**
 * @brief Setting for Bluetooth.
 *
 */
void bt_settings();

/**
 * @brief Receive input data from the gamepad and return it in PS2/3 gamepad array format.
 *
 * @param mount_joypad  Gamepad type (currently 2:KRC-5FH and 5:wiimote are available).
 * @param pre_val Previous received value (8 bytes, assuming union data).
 * @param polling Frame count for inquiry frequency.
 * @param joypad_reflesh 1:To reset the JOYPAD's received button data to 0 with this device
 *                       0:perform logical addition without resetting .(usually 1)
 * @return uint64_t
 */
uint64_t joypad_read(int mount_joypad, uint64_t pre_val, int polling, bool joypad_reflesh);

/**
 * @brief Receive input values from the wiimote
 *        and store them in pad_btn.
 */
uint16_t pad_wiimote_receive();

/**
 * @brief Initialize sensors.
 *
 * @param[in] int Number of mounted imuahrs.
 *            0:off, 1:MPU6050(GY-521), 2:MPU9250(GY-6050/GY-9250) 3:BNO055
 *            Currently only 3:bno055 is available.
 */
void init_imuahrs(int mount_imuahrs);

/**
 * @brief Read the data in bno055 and write it to bno055_read[].
 *
 * @param[in] void *args Pointer used by the system for thread processing.
 */
void Core1_bno055_r(void *args);

/**
 * @brief Show data of joypad's input on serial monitor.
 *
 * @param[in] ushort Array consisting of 4 pieces.
 */
void monitor_joypad(ushort *arr);

/**
 * @brief Monitor expected sequential number / received sequential number.
 *
 * @param[in] int Expected sequential number.
 * @param[in] int Received sequential number.
 */
void monitor_seq_num(int exp, int rsvd, bool monitor_seq);

/**
 * @brief Execute mastercommands.
 *
 */
void execute_MasterCommand();

/**
 * @brief Powering off all servos.
 *
 */
void servo_all_off();

/**
 * @brief Resetting the origin of the yaw axis.
 *        Use MOUNT_IMUAHRS for device model detection.
 *        0:none, 1:MPU6050(GY-521), 2:MPU9250(GY-6050/GY-9250) 3:BNO055
 *        Now only 3:BNO055 is available
 */
void setyawcenter();

#endif
