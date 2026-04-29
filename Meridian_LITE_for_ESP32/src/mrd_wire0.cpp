// mrd_wire0.cpp
// I2C wire0 関数の実装

// ヘッダファイルの読み込み
#include "mrd_wire0.h"

// ライブラリ導入
#include <Wire.h> // I2C通信のためのWireライブラリ導入

// センサーライブラリの条件付きインクルード
#if MOUNT_IMUAHRS == IMUAHRS_BNO055
#include <Adafruit_BNO055.h>
#endif
#if MOUNT_IMUAHRS == IMUAHRS_MPU6050
#include <MPU6050_6Axis_MotionApps20.h>
#endif

extern SemaphoreHandle_t ahrs_mutex; // AHRSデータアクセス用mutex

//==================================================================================================
//  I2C wire0 関連の処理
//==================================================================================================
// 6軸or9軸センサーの値
struct AhrsValue {
#if MOUNT_IMUAHRS == IMUAHRS_BNO055
  Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire); // BNO055のインスタンス
#endif
#if MOUNT_IMUAHRS == IMUAHRS_MPU6050
  MPU6050 mpu6050;     // MPU6050のインスタンス
  Quaternion q;        // [w, x, y, z]         quaternion container
  VectorFloat gravity; // [x, y, z]            gravity vector
  VectorInt16 aa;      // [x, y, z]            加速度センサの測定値
  VectorInt16 gyro;    // [x, y, z]            角速度センサの測定値
  VectorInt16 mag;     // [x, y, z]            磁力センサの測定値
#endif

  uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
  uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
  uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
  uint8_t fifoBuffer[64]; // FIFO storage buffer
  float ypr[3];           // [roll, pitch, yaw]   roll/pitch/yaw container and gravity vector
  float yaw_origin = 0;   // ヨー軸の補正センター値
  float yaw_source = 0;   // ヨー軸のソースデータ保持用

  float read[16]; // mpuからの読み込んだ一次データacc_x,y,z,gyro_x,y,z,mag_x,y,z,gr_x,y,z,rpy_r,p,y,temp

  float zeros[16] = {0};               // リセット用
  float ave_data[16];                  // 上記の移動平均値を入れる
  float result[16];                    // 加工後の最新のmpuデータ(二次データ)
  float stock_data[IMUAHRS_STOCK][16]; // 上記の移動平均値計算用のデータストック
  int stock_count = 0;                 // 上記の移動平均値計算用のデータストックを輪番させる時の変数
  long temperature;                    // センサの温度測定値
};

// 6軸or9軸センサーの値
AhrsValue m_ahrs;

//------------------------------------------------------------------------------------
//  初期設定
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
/// @return DMP初期化が成功した場合はtrue, 失敗した場合はfalse
bool mrd_wire0_init_mpu6050_dmp() {
#if MOUNT_IMUAHRS == IMUAHRS_MPU6050 // MPU6050_6Axis_MotionApps20
  m_ahrs.mpu6050.initialize();
  m_ahrs.devStatus = m_ahrs.mpu6050.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  m_ahrs.mpu6050.setXAccelOffset(-1745);
  m_ahrs.mpu6050.setYAccelOffset(-1034);
  m_ahrs.mpu6050.setZAccelOffset(966);
  m_ahrs.mpu6050.setXGyroOffset(176);
  m_ahrs.mpu6050.setYGyroOffset(-6);
  m_ahrs.mpu6050.setZGyroOffset(-25);

  // make sure it worked (returns 0 if so)
  if (m_ahrs.devStatus == 0) {
    m_ahrs.mpu6050.CalibrateAccel(6);
    m_ahrs.mpu6050.CalibrateGyro(6);
    m_ahrs.mpu6050.setDMPEnabled(true);
    m_ahrs.packetSize = m_ahrs.mpu6050.dmpGetFIFOPacketSize();
    Serial.println("MPU6050 OK.");
    return true;
  }
#endif
  Serial.println("IMU/AHRS DMP Initialization FAILED!");
  return false;
}

/// @brief BNO055センサの初期化を試みる
/// @return BNO055初期化が成功した場合はtrue, 検出できなかった場合はfalse
bool mrd_wire0_init_bno055() {
#if MOUNT_IMUAHRS == IMUAHRS_BNO055 // Adafruit_BNO055
  if (!m_ahrs.bno.begin()) {
    Serial.println("No BNO055 detected ... Check your wiring or I2C ADDR!");
    return false;
  } else {
    Serial.println("BNO055 mounted.");
    delay(50);
    m_ahrs.bno.setExtCrystalUse(false);
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
/// @param a_pinSDA SDAピン番号. a_pinSCLと共に省略可能.
/// @param a_pinSCL SCLピン番号. a_pinSDAと共に省略可能.
/// @return センサが正しく初期化された場合はtrue, それ以外はfalse
bool mrd_wire0_setup(ImuAhrsType a_imuahrs_type, int a_i2c0_speed, int a_pinSDA, int a_pinSCL) {
  if (a_imuahrs_type > 0) { // 何らかのセンサが搭載されている
    if (a_pinSDA == -1 && a_pinSCL == -1) {
      mrd_wire0_init_i2c(a_i2c0_speed);
    } else {
      mrd_wire0_init_i2c(a_i2c0_speed, a_pinSDA, a_pinSCL);
    }
  }

  if (a_imuahrs_type == MPU6050_IMU) { // MPU6050
    return mrd_wire0_init_mpu6050_dmp();
  } else if (a_imuahrs_type == MPU9250_IMU) { // MPU9250の場合
    // mrd_wire_init_mpu9250_dmp(m_ahrs)
    return false;
  } else if (a_imuahrs_type == BNO055_AHRS) { // BNO055の場合
    return mrd_wire0_init_bno055();
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
  float local_read[16];             // ローカルバッファ
#if MOUNT_IMUAHRS == IMUAHRS_BNO055 // Adafruit_BNO055
  while (1) {
    // mutex保護下でセンサ読み取りと共有バッファへの書き込みを実行
    if (xSemaphoreTake(ahrs_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
      // 加速度値を取得 - VECTOR_ACCELEROMETER - m/s^2
      imu::Vector<3> accelerometer = m_ahrs.bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
      local_read[0] = (float)accelerometer.x();
      local_read[1] = (float)accelerometer.y();
      local_read[2] = (float)accelerometer.z();

      // ジャイロ値を取得 - VECTOR_GYROSCOPE - rad/s
      imu::Vector<3> gyroscope = m_ahrs.bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
      local_read[3] = gyroscope.x();
      local_read[4] = gyroscope.y();
      local_read[5] = gyroscope.z();

      // 磁力計値を取得 - VECTOR_MAGNETOMETER - uT
      imu::Vector<3> magnetometer = m_ahrs.bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
      local_read[6] = magnetometer.x();
      local_read[7] = magnetometer.y();
      local_read[8] = magnetometer.z();

      // センサフュージョンによる推定姿勢を取得 - VECTOR_EULER - degrees
      imu::Vector<3> euler = m_ahrs.bno.getVector(Adafruit_BNO055::VECTOR_EULER);
      local_read[12] = euler.y();                    // DMP_ROLL 推定値
      local_read[13] = euler.z();                    // DMP_PITCH 推定値
      float yaw_tmp = euler.x() - m_ahrs.yaw_origin; // DMP_YAW 推定値
      if (yaw_tmp >= 180) {
        yaw_tmp = yaw_tmp - 360;
      } else if (yaw_tmp < -180) {
        yaw_tmp = yaw_tmp + 360;
      }
      local_read[14] = yaw_tmp;

      // 共有バッファにコピー
      memcpy(m_ahrs.read, local_read, sizeof(float) * 16);
      m_ahrs.yaw_source = euler.x();
      m_ahrs.ypr[0] = local_read[14];
      m_ahrs.ypr[1] = local_read[13];
      m_ahrs.ypr[2] = local_read[12];

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
bool mrd_wire0_read_ahrs_i2c(MrdFlags &a_flg) { // wireTimer0.begin引数にvoidが必要
#if MOUNT_IMUAHRS == IMUAHRS_MPU6050            // MPU6050_6Axis_MotionApps20

  if (MOUNT_IMUAHRS == MPU6050_IMU) {                                // MPU6050
    if (m_ahrs.mpu6050.dmpGetCurrentFIFOPacket(m_ahrs.fifoBuffer)) { // Get new data
      m_ahrs.mpu6050.dmpGetQuaternion(&m_ahrs.q, m_ahrs.fifoBuffer);
      m_ahrs.mpu6050.dmpGetGravity(&m_ahrs.gravity, &m_ahrs.q);
      m_ahrs.mpu6050.dmpGetYawPitchRoll(m_ahrs.ypr, &m_ahrs.q, &m_ahrs.gravity);

      // acceleration values
      m_ahrs.mpu6050.dmpGetAccel(&m_ahrs.aa, m_ahrs.fifoBuffer);
      m_ahrs.read[0] = (float)m_ahrs.aa.x;
      m_ahrs.read[1] = (float)m_ahrs.aa.y;
      m_ahrs.read[2] = (float)m_ahrs.aa.z;

      // gyro values
      m_ahrs.mpu6050.dmpGetGyro(&m_ahrs.gyro, m_ahrs.fifoBuffer);
      m_ahrs.read[3] = (float)m_ahrs.gyro.x;
      m_ahrs.read[4] = (float)m_ahrs.gyro.y;
      m_ahrs.read[5] = (float)m_ahrs.gyro.z;

      // magnetic field values
      m_ahrs.read[6] = (float)m_ahrs.mag.x;
      m_ahrs.read[7] = (float)m_ahrs.mag.y;
      m_ahrs.read[8] = (float)m_ahrs.mag.z;

      // Estimated gravity DMP value.
      m_ahrs.read[9] = m_ahrs.gravity.x;
      m_ahrs.read[10] = m_ahrs.gravity.y;
      m_ahrs.read[11] = m_ahrs.gravity.z;

      // Estimated heading value using DMP.
      m_ahrs.read[12] = m_ahrs.ypr[2] * 180 / M_PI;                       // Estimated DMP_ROLL
      m_ahrs.read[13] = m_ahrs.ypr[1] * 180 / M_PI;                       // Estimated DMP_PITCH
      m_ahrs.read[14] = (m_ahrs.ypr[0] * 180 / M_PI) - m_ahrs.yaw_origin; // Estimated DMP_YAW

      // Temperature
      m_ahrs.read[15] = 0; // Not implemented.

      if (a_flg.imuahrs_available) {
        memcpy(m_ahrs.result, m_ahrs.read, sizeof(float) * 16);
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
bool meriput90_ahrs(Meridim90Union &a_meridim, int a_type, MERIDIANFLOW::Meridian &mrd, MrdFlags &a_flg) {
  if (a_type == BNO055_AHRS) {
    float local_copy[16];

    // mutex保護下でAHRSデータをコピー
    if (xSemaphoreTake(ahrs_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      memcpy(local_copy, m_ahrs.read, sizeof(float) * 16);
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

void mrd_wire0_calibrate_yaw_origin() {
  m_ahrs.yaw_origin = m_ahrs.yaw_source;
}
