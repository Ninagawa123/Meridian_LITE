#ifndef __MERIDIAN_WIRE0_H__
#define __MERIDIAN_WIRE0_H__

#include "../config.h"
#include "mrd_types.h"

//==================================================================================================
//  I2C wire0 関連の処理
//==================================================================================================

/// @brief Wire0 I2C通信を初期化し, 指定されたクロック速度で設定する.
bool mrd_wire0_init_i2c(int a_i2c0_speed, int a_pinSDA = -1, int a_pinSCL = -1);

/// @brief MPU6050センサーのDMPを初期化し, ジャイロ/加速度オフセットを設定する.
bool mrd_wire0_init_mpu6050_dmp();

/// @brief BNO055センサーを初期化する.
bool mrd_wire0_init_bno055(AhrsValue &a_ahrs);

/// @brief 指定されたIMU/AHRSタイプに応じて適切なセンサの初期化を行う.
bool mrd_wire0_setup(ImuAhrsType a_imuahrs_type, int a_i2c0_speed, AhrsValue &a_ahrs,
                     int a_pinSDA = -1, int a_pinSCL = -1);

/// @brief BNO055からI2C経由でデータを読み取るCore0スレッド関数.
void mrd_wire0_Core0_bno055_r(void *args);

/// @brief MPU6050からI2C経由でAHRSデータを読み取る.
bool mrd_wire0_read_ahrs_i2c(AhrsValue &a_ahrs);

/// @brief 計測したAHRSデータをMeridim配列に書き込む.
bool meriput90_ahrs(Meridim90Union &a_meridim, float a_ahrs_result[], int a_type);

#endif // __MERIDIAN_WIRE0_H__
