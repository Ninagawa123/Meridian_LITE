#ifndef __MERIDIAN_WIRE0_H__
#define __MERIDIAN_WIRE0_H__

// ヘッダファイルの読み込み
#include "config.h"
#include "main.h"

// ライブラリ導入
#include <Wire.h>

//================================================================================================================
//  I2C wire0 関連の処理
//  ---------------------------------------------------------------------------------------
//================================================================================================================

//------------------------------------------------------------------------------------
//  初期設定
//------------------------------------------------------------------------------------

/// @brief Wire0 I2C通信を初期化し, 指定されたクロック速度で設定する.
/// @param a_i2c0_speed I2C通信のクロック速度です.
/// @param a_pinSDA SDAのピン番号. 下記と合わせて省略可.
/// @param a_pinSCL SCLのピン番号. 上記と合わせて省略可.
bool mrd_wire0_init_i2c(int a_i2c0_speed, int a_pinSDA = -1, int a_pinSCL = -1) {
  Serial.print("Initializing wire0 I2C... ");
  if (a_pinSDA == -1 && a_pinSCL == -1) {
    Wire.begin();
  } else {
    Wire.begin(a_pinSDA, a_pinSCL);
  }
  Wire.setClock(a_i2c0_speed);
  return true;
}

/// @brief MPU6050センサーのDMP（デジタルモーションプロセッサ）を初期化し,
/// ジャイロスコープと加速度センサーのオフセットを設定する.
/// @param a_ahrs AHRSの値を保持する構造体.
/// @return DMPの初期化が成功した場合はtrue, 失敗した場合はfalseを返す.
bool mrd_wire0_init_mpu6050_dmp(AhrsValue &a_ahrs) {
  a_ahrs.mpu6050.initialize();
  a_ahrs.devStatus = a_ahrs.mpu6050.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  a_ahrs.mpu6050.setXAccelOffset(-1745);
  a_ahrs.mpu6050.setYAccelOffset(-1034);
  a_ahrs.mpu6050.setZAccelOffset(966);
  a_ahrs.mpu6050.setXGyroOffset(176);
  a_ahrs.mpu6050.setYGyroOffset(-6);
  a_ahrs.mpu6050.setZGyroOffset(-25);

  // make sure it worked (returns 0 if so)
  if (a_ahrs.devStatus == 0) {
    a_ahrs.mpu6050.CalibrateAccel(6);
    a_ahrs.mpu6050.CalibrateGyro(6);
    a_ahrs.mpu6050.setDMPEnabled(true);
    a_ahrs.packetSize = a_ahrs.mpu6050.dmpGetFIFOPacketSize();
    Serial.println("MPU6050 OK.");
    return true;
  }
  Serial.println("IMU/AHRS DMP Initialization FAILED!");
  return false;
}

/// @brief BNO055センサーの初期化を試みます.
/// @param a_ahrs AHRSの値を保持する構造体.
/// @return BNO055センサーの初期化が成功した場合はtrue, それ以外の場合はfalseを返す.
///         現在, この関数は常にfalseを返すように設定されています.
bool mrd_wire0_init_bno055(AhrsValue &a_ahrs) {
  if (!ahrs.bno.begin()) {
    Serial.println("No BNO055 detected ... Check your wiring or I2C ADDR!");
    return false;
  } else {
    Serial.println("BNO055 mounted.");
    delay(50);
    ahrs.bno.setExtCrystalUse(false);
    delay(10);
    return true;
  }
  // データの取得はセンサー用スレッドで実行
}

/// @brief 指定されたIMU/AHRSタイプに応じて適切なセンサの初期化を行います.
/// @param a_imuahrs_type 使用するセンサのタイプを示す列挙型です（MPU6050, MPU9250, BNO055）.
/// @param a_i2c0_speed I2C通信のクロック速度です.
/// @param a_ahrs AHRSの値を保持する構造体.
/// @param a_pinSDA SDAのピン番号.下記と合わせて省略可.
/// @param a_pinSCL SCLのピン番号.上記と合わせて省略可.
/// @return センサが正しく初期化された場合はtrueを, そうでない場合はfalseを返す.
bool mrd_wire0_setup(ImuAhrsType a_imuahrs_type, int a_i2c0_speed, AhrsValue &a_ahrs,
                     int a_pinSDA = -1, int a_pinSCL = -1) {
  if (a_imuahrs_type > 0) // 何らかのセンサを搭載
  {
    if (a_pinSDA == -1 && a_pinSCL == -1) {
      mrd_wire0_init_i2c(a_i2c0_speed);
    } else {
      mrd_wire0_init_i2c(a_i2c0_speed, a_pinSDA, a_pinSCL);
    }
  }

  if (a_imuahrs_type == MPU6050_IMU) // MPU6050
  {
    return mrd_wire0_init_mpu6050_dmp(a_ahrs);
  } else if (a_imuahrs_type == MPU9250_IMU) // MPU9250の場合
  {
    // mrd_wire_init_mpu9250_dmp(a_ahrs)
    return false;
  } else if (a_imuahrs_type == BNO055_AHRS) // BNO055の場合
  {
    return mrd_wire0_init_bno055(a_ahrs);
  }

  Serial.println("No IMU/AHRS sensor mounted.");
  return false;
}

//------------------------------------------------------------------------------------
//  センサデータの取得処理
//------------------------------------------------------------------------------------

/// @brief bno055からI2C経由でデータを読み取るスレッド用関数. IMUAHRS_INTERVALの間隔で実行する.
void mrd_wire0_Core0_bno055_r(void *args) {
  while (1) {
    // 加速度センサ値の取得と表示 - VECTOR_ACCELEROMETER - m/s^2
    imu::Vector<3> accelermetor = ahrs.bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    ahrs.read[0] = (float)accelermetor.x();
    ahrs.read[1] = (float)accelermetor.y();
    ahrs.read[2] = (float)accelermetor.z();

    // ジャイロセンサ値の取得 - VECTOR_GYROSCOPE - rad/s
    imu::Vector<3> gyroscope = ahrs.bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    ahrs.read[3] = gyroscope.x();
    ahrs.read[4] = gyroscope.y();
    ahrs.read[5] = gyroscope.z();

    // 磁力センサ値の取得と表示  - VECTOR_MAGNETOMETER - uT
    imu::Vector<3> magnetmetor = ahrs.bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
    ahrs.read[6] = magnetmetor.x();
    ahrs.read[7] = magnetmetor.y();
    ahrs.read[8] = magnetmetor.z();

    // センサフュージョンによる方向推定値の取得と表示 - VECTOR_EULER - degrees
    imu::Vector<3> euler = ahrs.bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    ahrs.read[12] = euler.y();                   // DMP_ROLL推定値
    ahrs.read[13] = euler.z();                   // DMP_PITCH推定値
    ahrs.yaw_source = euler.x();                 // ヨー軸のソースデータ保持
    float yaw_tmp = euler.x() - ahrs.yaw_origin; // DMP_YAW推定値
    if (yaw_tmp >= 180) {
      yaw_tmp = yaw_tmp - 360;
    } else if (yaw_tmp < -180) {
      yaw_tmp = yaw_tmp + 360;
    }
    ahrs.read[14] = yaw_tmp; // DMP_YAW推定値
    ahrs.ypr[0] = ahrs.read[14];
    ahrs.ypr[1] = ahrs.read[13];
    ahrs.ypr[2] = ahrs.read[12];

    // センサフュージョンの方向推定値のクオータニオン
    // imu::Quaternion quat = bno.getQuat();

    // Serial.print("qW: ");
    // Serial.print(quat.w(), 4);
    // Serial.print(" qX: ");
    // Serial.print(quat.x(), 4);
    // Serial.print(" qY: ");
    // Serial.print(quat.y(), 4);
    // Serial.print(" qZ: ");
    // Serial.println(quat.z(), 4);

    // キャリブレーションのステータスの取得と表示
    // uint8_t system, gyro, accel, mag = 0;
    // bno.getCalibration(&system, &gyro, &accel, &mag);
    // Serial.print("CALIB Sys:");
    // Serial.print(system, DEC);
    // Serial.print(", Gy");
    // Serial.print(gyro, DEC);
    // Serial.print(", Ac");
    // Serial.print(accel, DEC);
    // Serial.print(", Mg");
    // Serial.println(mag, DEC);

    delay(IMUAHRS_INTERVAL);
  }
}

/// @brief AHRSセンサーからI2C経由でデータを読み取る関数.
/// MPU6050, MPU9250を想定していますが, MPU9250は未実装.
/// 各データは`ahrs.read`配列に格納され, 利用可能な場合は`ahrs.result`にコピーされる.
bool mrd_wire0_read_ahrs_i2c(AhrsValue a_ahrs) { // ※wireTimer0.beginの引数のためvoid必須
  if (MOUNT_IMUAHRS == MPU6050_IMU) {            // MPU6050
    if (a_ahrs.mpu6050.dmpGetCurrentFIFOPacket(ahrs.fifoBuffer)) { // Get new data
      a_ahrs.mpu6050.dmpGetQuaternion(&ahrs.q, ahrs.fifoBuffer);
      a_ahrs.mpu6050.dmpGetGravity(&ahrs.gravity, &ahrs.q);
      a_ahrs.mpu6050.dmpGetYawPitchRoll(ahrs.ypr, &ahrs.q, &ahrs.gravity);

      // acceleration values
      a_ahrs.mpu6050.dmpGetAccel(&ahrs.aa, ahrs.fifoBuffer);
      a_ahrs.read[0] = (float)ahrs.aa.x;
      a_ahrs.read[1] = (float)ahrs.aa.y;
      a_ahrs.read[2] = (float)ahrs.aa.z;

      // gyro values
      a_ahrs.mpu6050.dmpGetGyro(&ahrs.gyro, ahrs.fifoBuffer);
      a_ahrs.read[3] = (float)ahrs.gyro.x;
      a_ahrs.read[4] = (float)ahrs.gyro.y;
      a_ahrs.read[5] = (float)ahrs.gyro.z;

      // magnetic field values
      a_ahrs.read[6] = (float)ahrs.mag.x;
      a_ahrs.read[7] = (float)ahrs.mag.y;
      a_ahrs.read[8] = (float)ahrs.mag.z;

      // Estimated gravity DMP value.
      a_ahrs.read[9] = ahrs.gravity.x;
      a_ahrs.read[10] = ahrs.gravity.y;
      a_ahrs.read[11] = ahrs.gravity.z;

      // Estimated heading value using DMP.
      a_ahrs.read[12] = ahrs.ypr[2] * 180 / M_PI;                     // Estimated DMP_ROLL
      a_ahrs.read[13] = ahrs.ypr[1] * 180 / M_PI;                     // Estimated DMP_PITCH
      a_ahrs.read[14] = (ahrs.ypr[0] * 180 / M_PI) - ahrs.yaw_origin; // Estimated DMP_YAW

      // Temperature
      a_ahrs.read[15] = 0; // Not implemented.

      if (flg.imuahrs_available) {
        memcpy(a_ahrs.result, a_ahrs.read, sizeof(float) * 16);
      }
      return true;
    } else {
      return false;
    }
  } else if (MOUNT_IMUAHRS == MPU9250_IMU) { // MPU9250
    return false;
  } else {
    return false;
  }
}

//------------------------------------------------------------------------------------
//  IntervalTimerTimerのスタート
//------------------------------------------------------------------------------------

/// @brief AHRSセンサーからデータを読み取るための関数を実行する.
void mrd_wire0_run() { // ※IntervalTimer用の関数のためvoidのみ可
  mrd_wire0_read_ahrs_i2c(ahrs);
}

/// @brief インターバルタイマーを設定して, 定期的にAHRSセンサーからのデータ読み取りを行う.
/// @param a_check タイマーを開始するかどうかの条件を示すブール値です.
/// @return タイマーが正しく開始された場合はtrue, それ以外の場合はfalseを返す.
bool mrd_wire0_startIntervalTimer(bool a_check) {
  if (a_check) {
    // bool rlst = wireTimer0.begin(mrd_wire0_run, IMUAHRS_INTERVAL * 1000); //
    // インターバルはマイクロ秒指定
    //  wireTimer.priority(90);
    // return rlst;
    return false;
  }
  return false;
}

//------------------------------------------------------------------------------------
//  各種オペレーション
//------------------------------------------------------------------------------------

/// @brief 指定されたIMU/AHRSタイプに基づいて, 計測したAHRSデータをMeridim配列に格納する.
/// @param a_imuahrs_type 使用するセンサのタイプを示す列挙型です（MPU6050, MPU9250, BNO055）.
/// @param a_meridim Meridim配列. 計測データを格納する.
/// @param a_ahrs_result AHRSから読み取った結果を格納した配列.
/// @return データの書き込みが成功した場合はtrue, それ以外の場合はfalseを返す.
bool mrd_wire0_copy_ahrs_data(ImuAhrsType a_imuahrs_type, short a_meridim[],
                              float a_ahrs_result[]) {
  if (a_imuahrs_type == MPU6050_IMU) {
    flg.imuahrs_available = false;
    a_meridim[2] = mrd.float2HfShort(a_ahrs_result[0]);   // IMU/AHRS_acc_x
    a_meridim[3] = mrd.float2HfShort(a_ahrs_result[1]);   // IMU/AHRS_acc_y
    a_meridim[4] = mrd.float2HfShort(a_ahrs_result[2]);   // IMU/AHRS_acc_z
    a_meridim[5] = mrd.float2HfShort(a_ahrs_result[3]);   // IMU/AHRS_gyro_x
    a_meridim[6] = mrd.float2HfShort(a_ahrs_result[4]);   // IMU/AHRS_gyro_y
    a_meridim[7] = mrd.float2HfShort(a_ahrs_result[5]);   // IMU/AHRS_gyro_z
    a_meridim[8] = mrd.float2HfShort(a_ahrs_result[6]);   // IMU/AHRS_mag_x
    a_meridim[9] = mrd.float2HfShort(a_ahrs_result[7]);   // IMU/AHRS_mag_y
    a_meridim[10] = mrd.float2HfShort(a_ahrs_result[8]);  // IMU/AHRS_mag_z
    a_meridim[11] = mrd.float2HfShort(a_ahrs_result[15]); // temperature
    a_meridim[12] = mrd.float2HfShort(a_ahrs_result[12]); // DMP_ROLL推定値
    a_meridim[13] = mrd.float2HfShort(a_ahrs_result[13]); // DMP_PITCH推定値
    a_meridim[14] = mrd.float2HfShort(a_ahrs_result[14]); // DMP_YAW推定値
    flg.imuahrs_available = true;
    return true;
  } else if (a_imuahrs_type == MPU9250_IMU) { // 未設定
    return false;
  } else if (a_imuahrs_type == BNO055_AHRS) {
    for (int i = MRD_ACC_X; i <= MRD_DIR_YAW; i++) {
      s_udp_meridim.sval[i] = mrd.float2HfShort(ahrs.read[i]); // DMP_ROLL推定値
    }
    return false;
  }
  return false;
}

#endif // __MERIDIAN_WIRE0_H__
