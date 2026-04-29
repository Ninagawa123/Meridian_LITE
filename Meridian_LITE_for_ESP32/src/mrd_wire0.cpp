// mrd_wire0.cpp
// I2C wire0 関数の実装

#include "mrd_wire0.h"
#include <Meridian.h>

// main.hで定義された変数のextern宣言 (AhrsValue以外)
extern MrdFlags flg;
extern MERIDIANFLOW::Meridian mrd;

//==================================================================================================
//  I2C wire0 関数
//==================================================================================================

//------------------------------------------------------------------------------------
//  初期化
//------------------------------------------------------------------------------------

/// @brief 指定されたクロック速度でWire0 I2C通信を初期化する
/// @param a_i2c0_speed I2C通信クロック速度
/// @param a_pinSDA SDAピン番号. a_pinSCLと共に省略可能.
/// @param a_pinSCL SCLピン番号. a_pinSDAと共に省略可能.
/// @return 完了時にtrueを返す
bool mrd_wire0_init_i2c(int a_i2c0_speed, int a_pinSDA, int a_pinSCL) {
  Serial.print("Initializing wire0 I2C... ");
  if (a_pinSDA == -1 && a_pinSCL == -1) {
    Wire.begin();
  } else {
    Wire.begin(a_pinSDA, a_pinSCL);
  }
  Wire.setClock(a_i2c0_speed);
  return true;
}

/// @brief MPU6050センサのDMP (Digital Motion Processor) を初期化し,
///        ジャイロスコープと加速度センサのオフセットを設定する
/// @param a_ahrs AHRS値を保持する構造体
/// @return DMP初期化が成功した場合はtrue, 失敗した場合はfalse
bool mrd_wire0_init_mpu6050_dmp(AhrsValue &a_ahrs) {
#if MOUNT_IMUAHRS == 1 // MPU6050_6Axis_MotionApps20
  a_ahrs.mpu6050.initialize();
  a_ahrs.devStatus = a_ahrs.mpu6050.dmpInitialize();

  // 最小感度用にスケーリングされたジャイロオフセットを設定
  a_ahrs.mpu6050.setXAccelOffset(-1745);
  a_ahrs.mpu6050.setYAccelOffset(-1034);
  a_ahrs.mpu6050.setZAccelOffset(966);
  a_ahrs.mpu6050.setXGyroOffset(176);
  a_ahrs.mpu6050.setYGyroOffset(-6);
  a_ahrs.mpu6050.setZGyroOffset(-25);

  // 動作確認 (成功時は0を返す)
  if (a_ahrs.devStatus == 0) {
    a_ahrs.mpu6050.CalibrateAccel(6);
    a_ahrs.mpu6050.CalibrateGyro(6);
    a_ahrs.mpu6050.setDMPEnabled(true);
    a_ahrs.packetSize = a_ahrs.mpu6050.dmpGetFIFOPacketSize();
    Serial.println("MPU6050 OK.");
    return true;
  }
#endif
  Serial.println("IMU/AHRS DMP Initialization FAILED!");
  return false;
}

/// @brief BNO055センサの初期化を試みる
/// @param a_ahrs AHRS値を保持する構造体
/// @return BNO055初期化が成功した場合はtrue, 検出できなかった場合はfalse
bool mrd_wire0_init_bno055(AhrsValue &a_ahrs) {
#if MOUNT_IMUAHRS == 3 // Adafruit_BNO055
  if (!a_ahrs.bno.begin()) {
    Serial.println("No BNO055 detected ... Check your wiring or I2C ADDR!");
    return false;
  } else {
    Serial.println("BNO055 mounted.");
    delay(50);
    a_ahrs.bno.setExtCrystalUse(false);
    delay(10);
    return true;
  }
#else
  return false;
#endif
  // データ取得はセンサスレッドで実行される
}

/// @brief 指定されたIMU/AHRSタイプに基づいて適切なセンサを初期化する
/// @param a_imuahrs_type センサタイプ (0:なし, 1:MPU6050, 2:MPU9250, 3:BNO055)
/// @param a_i2c0_speed I2C通信クロック速度
/// @param a_ahrs AHRS値を保持する構造体
/// @param a_pinSDA SDAピン番号. a_pinSCLと共に省略可能.
/// @param a_pinSCL SCLピン番号. a_pinSDAと共に省略可能.
/// @return センサが正しく初期化された場合はtrue, それ以外はfalse
bool mrd_wire0_setup(int a_imuahrs_type, int a_i2c0_speed, AhrsValue &a_ahrs,
                     int a_pinSDA, int a_pinSCL) {
  if (a_imuahrs_type > 0) // 何らかのセンサが搭載されている
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
  } else if (a_imuahrs_type == MPU9250_IMU) // MPU9250
  {
    // mrd_wire_init_mpu9250_dmp(a_ahrs)
    return false;
  } else if (a_imuahrs_type == BNO055_AHRS) // BNO055
  {
    return mrd_wire0_init_bno055(a_ahrs);
  }

  Serial.println("No IMU/AHRS sensor mounted.");
  return false;
}

//------------------------------------------------------------------------------------
//  センサデータ取得
//------------------------------------------------------------------------------------

/// @brief I2C経由でBNO055からデータを読み取るスレッド関数. IMUAHRS_INTERVAL間隔で実行.
/// @param args 未使用の引数
void mrd_wire0_Core0_bno055_r(void *args) {
  float local_read[16]; // ローカルバッファ
#if MOUNT_IMUAHRS == 3  // Adafruit_BNO055
  while (1) {
    // mutex保護下でセンサ読み取りと共有バッファへの書き込みを実行
    if (xSemaphoreTake(ahrs_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
      // 加速度値を取得 - VECTOR_ACCELEROMETER - m/s^2
      imu::Vector<3> accelerometer = ahrs.bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
      local_read[0] = (float)accelerometer.x();
      local_read[1] = (float)accelerometer.y();
      local_read[2] = (float)accelerometer.z();

      // ジャイロ値を取得 - VECTOR_GYROSCOPE - rad/s
      imu::Vector<3> gyroscope = ahrs.bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
      local_read[3] = gyroscope.x();
      local_read[4] = gyroscope.y();
      local_read[5] = gyroscope.z();

      // 磁力計値を取得 - VECTOR_MAGNETOMETER - uT
      imu::Vector<3> magnetometer = ahrs.bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
      local_read[6] = magnetometer.x();
      local_read[7] = magnetometer.y();
      local_read[8] = magnetometer.z();

      // センサフュージョンによる推定姿勢を取得 - VECTOR_EULER - degrees
      imu::Vector<3> euler = ahrs.bno.getVector(Adafruit_BNO055::VECTOR_EULER);
      local_read[12] = euler.y();                  // DMP_ROLL 推定値
      local_read[13] = euler.z();                  // DMP_PITCH 推定値
      float yaw_tmp = euler.x() - ahrs.yaw_origin; // DMP_YAW 推定値
      if (yaw_tmp >= 180) {
        yaw_tmp = yaw_tmp - 360;
      } else if (yaw_tmp < -180) {
        yaw_tmp = yaw_tmp + 360;
      }
      local_read[14] = yaw_tmp;

      // 共有バッファにコピー
      memcpy(ahrs.read, local_read, sizeof(float) * 16);
      ahrs.yaw_source = euler.x();
      ahrs.ypr[0] = local_read[14];
      ahrs.ypr[1] = local_read[13];
      ahrs.ypr[2] = local_read[12];

      xSemaphoreGive(ahrs_mutex);
    }

    delay(IMUAHRS_INTERVAL);
  }
#endif
}

/// @brief I2C経由でAHRSセンサからデータを読み取る.
///        MPU6050, MPU9250用だが, MPU9250は未実装.
///        各データはahrs.read配列に格納され, 利用可能な場合ahrs.resultにコピーされる.
/// @param a_ahrs AHRS値を保持する構造体
/// @return 成功時はtrue, 失敗時はfalse
bool mrd_wire0_read_ahrs_i2c(AhrsValue &a_ahrs) {                    // wireTimer0.begin引数にvoidが必要
#if MOUNT_IMUAHRS == 1                                               // MPU6050_6Axis_MotionApps20
  if (MOUNT_IMUAHRS == MPU6050_IMU) {                                // MPU6050
    if (a_ahrs.mpu6050.dmpGetCurrentFIFOPacket(a_ahrs.fifoBuffer)) { // 新しいデータを取得
      a_ahrs.mpu6050.dmpGetQuaternion(&a_ahrs.q, a_ahrs.fifoBuffer);
      a_ahrs.mpu6050.dmpGetGravity(&a_ahrs.gravity, &a_ahrs.q);
      a_ahrs.mpu6050.dmpGetYawPitchRoll(a_ahrs.ypr, &a_ahrs.q, &a_ahrs.gravity);

      // 加速度値
      a_ahrs.mpu6050.dmpGetAccel(&a_ahrs.aa, a_ahrs.fifoBuffer);
      a_ahrs.read[0] = (float)a_ahrs.aa.x;
      a_ahrs.read[1] = (float)a_ahrs.aa.y;
      a_ahrs.read[2] = (float)a_ahrs.aa.z;

      // ジャイロ値
      a_ahrs.mpu6050.dmpGetGyro(&a_ahrs.gyro, a_ahrs.fifoBuffer);
      a_ahrs.read[3] = (float)a_ahrs.gyro.x;
      a_ahrs.read[4] = (float)a_ahrs.gyro.y;
      a_ahrs.read[5] = (float)a_ahrs.gyro.z;

      // 磁場値
      a_ahrs.read[6] = (float)a_ahrs.mag.x;
      a_ahrs.read[7] = (float)a_ahrs.mag.y;
      a_ahrs.read[8] = (float)a_ahrs.mag.z;

      // DMP推定重力値
      a_ahrs.read[9] = a_ahrs.gravity.x;
      a_ahrs.read[10] = a_ahrs.gravity.y;
      a_ahrs.read[11] = a_ahrs.gravity.z;

      // DMP推定姿勢値
      a_ahrs.read[12] = a_ahrs.ypr[2] * 180 / M_PI;                       // DMP_ROLL 推定値
      a_ahrs.read[13] = a_ahrs.ypr[1] * 180 / M_PI;                       // DMP_PITCH 推定値
      a_ahrs.read[14] = (a_ahrs.ypr[0] * 180 / M_PI) - a_ahrs.yaw_origin; // DMP_YAW 推定値

      // 温度
      a_ahrs.read[15] = 0; // 未実装

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
#else
  return false;
#endif
}

//------------------------------------------------------------------------------------
//  meriput
//------------------------------------------------------------------------------------

/// @brief 指定されたIMU/AHRSタイプに基づいて測定されたAHRSデータを読み取る
/// @param a_meridim Meridim配列共用体. 参照渡し.
/// @param a_ahrs_result AHRSから読み取った結果を格納する配列
/// @param a_type センサタイプのenum (MPU6050, MPU9250, BNO055)
/// @return データ書き込みが成功した場合はtrue, それ以外はfalse
bool meriput90_ahrs(Meridim90Union &a_meridim, float a_ahrs_result[], int a_type) {
  if (a_type == BNO055_AHRS) {
    float local_copy[16];

    // mutex保護下でAHRSデータをコピー
    if (xSemaphoreTake(ahrs_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      memcpy(local_copy, a_ahrs_result, sizeof(float) * 16);
      xSemaphoreGive(ahrs_mutex);
    } else {
      return false; // mutexを取得できなかった場合, このフレームをスキップ
    }

    a_meridim.sval[2] = mrd.float2HfShort(local_copy[0]);   // IMU/AHRS_acc_x
    a_meridim.sval[3] = mrd.float2HfShort(local_copy[1]);   // IMU/AHRS_acc_y
    a_meridim.sval[4] = mrd.float2HfShort(local_copy[2]);   // IMU/AHRS_acc_z
    a_meridim.sval[5] = mrd.float2HfShort(local_copy[3]);   // IMU/AHRS_gyro_x
    a_meridim.sval[6] = mrd.float2HfShort(local_copy[4]);   // IMU/AHRS_gyro_y
    a_meridim.sval[7] = mrd.float2HfShort(local_copy[5]);   // IMU/AHRS_gyro_z
    a_meridim.sval[8] = mrd.float2HfShort(local_copy[6]);   // IMU/AHRS_mag_x
    a_meridim.sval[9] = mrd.float2HfShort(local_copy[7]);   // IMU/AHRS_mag_y
    a_meridim.sval[10] = mrd.float2HfShort(local_copy[8]);  // IMU/AHRS_mag_z
    a_meridim.sval[11] = mrd.float2HfShort(local_copy[15]); // 温度
    a_meridim.sval[12] = mrd.float2HfShort(local_copy[12]); // DMP_ROLL 推定値
    a_meridim.sval[13] = mrd.float2HfShort(local_copy[13]); // DMP_PITCH 推定値
    a_meridim.sval[14] = mrd.float2HfShort(local_copy[14]); // DMP_YAW 推定値
    return true;
  }
  return false;
}
