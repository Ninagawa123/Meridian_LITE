#ifndef __MERIDIAN_WIRE0_H__
#define __MERIDIAN_WIRE0_H__

// ヘッダファイルの読み込み
#include "config.h"
#include "main.h"

// ライブラリ導入
#include <Wire.h>

//==================================================================================================
//  I2C wire0 関連の処理
//==================================================================================================

//------------------------------------------------------------------------------------
//  初期設定
//------------------------------------------------------------------------------------

/// @brief Wire0 I2C通信を初期化し, 指定されたクロック速度で設定する.
/// @param a_i2c0_speed I2C通信のクロック速度です.
/// @param a_pin_sda SDAのピン番号. 下記と合わせて省略可.
/// @param a_pin_scl SCLのピン番号. 上記と合わせて省略可.
bool mrd_wire0_i2c_init(int a_i2c0_speed, int a_pin_sda = -1, int a_pin_scl = -1) {
  Serial.print("Initializing wire0 I2C... ");
  if (a_pin_sda == -1 && a_pin_scl == -1) {
    Wire.begin();
  } else {
    Wire.begin(a_pin_sda, a_pin_scl);
  }
  Wire.setClock(a_i2c0_speed);
  return true;
}

/// @brief MPU6050センサーのDMP(デジタルモーションプロセッサ)を初期化し,
///        ジャイロスコープと加速度センサーのオフセットを設定する.
/// @param a_ahrs AHRSの値を保持する構造体.
/// @return DMPの初期化が成功した場合はtrue, 失敗した場合はfalseを返す.
bool mrd_wire0_mpu6050_dmp_init(AhrsValue &a_ahrs) {
  a_ahrs.mpu6050.initialize();
  a_ahrs.dev_status = a_ahrs.mpu6050.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  a_ahrs.mpu6050.setXAccelOffset(-1745);
  a_ahrs.mpu6050.setYAccelOffset(-1034);
  a_ahrs.mpu6050.setZAccelOffset(966);
  a_ahrs.mpu6050.setXGyroOffset(176);
  a_ahrs.mpu6050.setYGyroOffset(-6);
  a_ahrs.mpu6050.setZGyroOffset(-25);

  // make sure it worked (returns 0 if so)
  if (a_ahrs.dev_status == 0) {
    a_ahrs.mpu6050.CalibrateAccel(6);
    a_ahrs.mpu6050.CalibrateGyro(6);
    a_ahrs.mpu6050.setDMPEnabled(true);
    a_ahrs.packet_size = a_ahrs.mpu6050.dmpGetFIFOPacketSize();
    Serial.println("MPU6050 OK.");
    return true;
  }
  Serial.println("IMU/AHRS DMP Initialization FAILED!");
  return false;
}

/// @brief BNO055センサーの初期化を試みます.
/// @param a_ahrs AHRSの値を保持する構造体.
/// @return BNO055センサーの初期化が成功した場合はtrue, それ以外の場合はfalseを返す.
bool mrd_wire0_bno055_init(AhrsValue &a_ahrs) {
  if (!a_ahrs.bno.begin()) {
    Serial.println("No BNO055 detected ... Check your wiring or I2C ADDR!");
    return false;
  } else {
    Serial.println("BNO055 mounted.");
    delay(50);
    a_ahrs.bno.setExtCrystalUse(false);
    delay(10);
    return true; // データの取得はセンサー用スレッドで実行
  }
}

/// @brief 指定されたIMU/AHRSタイプに応じて適切なセンサの初期化を行います.
/// @param a_imuahrs_type 使用するセンサのタイプを示す列挙型です(MPU6050_IMU, BNO055_AHRS. MPU9250_IMUは未対応).
/// @param a_i2c0_speed I2C通信のクロック速度です.
/// @param a_ahrs AHRSの値を保持する構造体.
/// @param a_pin_sda SDAのピン番号.下記と合わせて省略可.
/// @param a_pin_scl SCLのピン番号.上記と合わせて省略可.
/// @return センサが正しく初期化された場合はtrueを, そうでない場合はfalseを返す.
bool mrd_wire0_setup(ImuAhrsType a_imuahrs_type, int a_i2c0_speed, AhrsValue &a_ahrs,
                     int a_pin_sda = -1, int a_pin_scl = -1) {
  if (a_imuahrs_type > 0) // 何らかのセンサを搭載
  {
    mrd_wire0_i2c_init(a_i2c0_speed, a_pin_sda, a_pin_scl);
  }

  if (a_imuahrs_type == MPU6050_IMU) // MPU6050
  {
    return mrd_wire0_mpu6050_dmp_init(a_ahrs);
  } else if (a_imuahrs_type == MPU9250_IMU) // MPU9250の場合
  {
    Serial.println("MPU9250 is not supported.");
    return false;
  } else if (a_imuahrs_type == BNO055_AHRS) // BNO055の場合
  {
    return mrd_wire0_bno055_init(a_ahrs);
  }

  Serial.println("No IMU/AHRS sensor mounted.");
  return false;
}

//------------------------------------------------------------------------------------
//  センサデータの取得処理
//------------------------------------------------------------------------------------

/// @brief ahrs.read をCore0のセンサスレッドとメインループで受け渡すための排他用スピンロック.
portMUX_TYPE ahrs_mux = portMUX_INITIALIZER_UNLOCKED;

/// @brief 角度から補正センター値を引き, -180以上180未満に折り返す.
/// @param a_source センサの角度(degree).
/// @param a_origin 補正センター値(degree. MCMD_SENSOR_YAW_CALIB / MCMD_SENSOR_ALL_CALIB で設定).
/// @return 補正後の角度(degree).
float mrd_wire0_deg_from_origin(float a_source, float a_origin) {
  float deg_tmp = a_source - a_origin;
  if (deg_tmp >= 180) {
    deg_tmp -= 360;
  } else if (deg_tmp < -180) {
    deg_tmp += 360;
  }
  return deg_tmp;
}

/// @brief センサの読み取りの成否が変わったときだけ表示する(途絶えたとき, 戻ったとき).
/// @param a_name センサの名前.
/// @param a_ok 今回の読み取りが正常か.
/// @param a_ok_prev 前回の状態(呼び出し側で保持する).
void mrd_wire0_health_report(const char *a_name, bool a_ok, bool &a_ok_prev) {
  if (a_ok != a_ok_prev) {
    Serial.println(String(a_name) + (a_ok ? ": reading restored." : ": no response. Sensor values are not updated."));
    a_ok_prev = a_ok;
  }
}

/// @brief BNO055 がI2Cで応答するか(チップIDのレジスタ 0x00 が 0xA0 か).
///        Adafruit のライブラリは読み取りの失敗を返さないため, 読む前にこれで確かめる.
bool mrd_wire0_bno055_alive() {
  const uint8_t addr_tmp = 0x28; // ahrs.bno のI2Cアドレス
  Wire.beginTransmission(addr_tmp);
  Wire.write(0x00);
  if (Wire.endTransmission(false) != 0 || Wire.requestFrom(addr_tmp, uint8_t(1)) != 1) {
    return false;
  }
  return Wire.read() == 0xA0;
}

/// @brief bno055からI2C経由でデータを読み取るスレッド用関数. IMUAHRS_INTERVALの間隔で実行する.
///        I2Cの読み取りを先に済ませ, ahrs.read への書き込みだけを排他区間で行う.
///        BNO055 が応答しないあいだは値を更新しない(前回の値のまま. 表示で知らせる).
void mrd_wire0_bno055_task(void *a_args) {
  bool ok_prev_tmp = true;
  while (1) {
    const bool alive_tmp = mrd_wire0_bno055_alive();
    mrd_wire0_health_report("BNO055", alive_tmp, ok_prev_tmp);
    if (!alive_tmp) {
      delay(cfg.imuahrs_interval);
      continue;
    }
    // 加速度センサ値 - VECTOR_ACCELEROMETER - m/s^2
    imu::Vector<3> accelerometer_tmp = ahrs.bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    // ジャイロセンサ値 - VECTOR_GYROSCOPE - deg/s
    imu::Vector<3> gyroscope_tmp = ahrs.bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    // 磁力センサ値 - VECTOR_MAGNETOMETER - uT
    imu::Vector<3> magnetometer_tmp = ahrs.bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
    // センサフュージョンによる方向推定値 - VECTOR_EULER - degrees
    imu::Vector<3> euler_tmp = ahrs.bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    // 温度 - degC (BNO055 内蔵の温度センサ. 1℃単位)
    const int8_t temp_tmp = ahrs.bno.getTemp();

    // ライブラリの値は double. ahrs は float で持つ
    ahrs.yaw_source = float(euler_tmp.x()); // ソースデータ保持
    ahrs.roll_source = float(euler_tmp.y());
    ahrs.pitch_source = float(euler_tmp.z());
    const float yaw_tmp = mrd_wire0_deg_from_origin(ahrs.yaw_source, ahrs.yaw_origin); // 補正センターからの角度
    const float roll_tmp = mrd_wire0_deg_from_origin(ahrs.roll_source, ahrs.roll_origin);
    const float pitch_tmp = mrd_wire0_deg_from_origin(ahrs.pitch_source, ahrs.pitch_origin);

    portENTER_CRITICAL(&ahrs_mux);
    ahrs.read[0] = float(accelerometer_tmp.x());
    ahrs.read[1] = float(accelerometer_tmp.y());
    ahrs.read[2] = float(accelerometer_tmp.z());
    ahrs.read[3] = float(gyroscope_tmp.x());
    ahrs.read[4] = float(gyroscope_tmp.y());
    ahrs.read[5] = float(gyroscope_tmp.z());
    ahrs.read[6] = float(magnetometer_tmp.x());
    ahrs.read[7] = float(magnetometer_tmp.y());
    ahrs.read[8] = float(magnetometer_tmp.z());
    ahrs.read[12] = roll_tmp;  // DMP_ROLL推定値
    ahrs.read[13] = pitch_tmp; // DMP_PITCH推定値
    ahrs.read[14] = yaw_tmp;   // DMP_YAW推定値
    ahrs.read[15] = temp_tmp;  // 温度(degC)
    portEXIT_CRITICAL(&ahrs_mux);

    delay(cfg.imuahrs_interval);
  }
}

/// @brief MPU6050からI2C経由でDMPのデータを読み取り, ahrs.read に格納する.
///        I2Cの読み取りと計算を先に済ませ, ahrs.read への書き込みだけを排他区間で行う.
/// @param a_ahrs AHRSの値を保持する構造体.
/// @return 新しいデータを読み取れた場合はtrue, そうでない場合はfalseを返す.
bool mrd_wire0_mpu6050_read(AhrsValue &a_ahrs) {
  if (!a_ahrs.mpu6050.dmpGetCurrentFIFOPacket(a_ahrs.fifo_buffer)) { // 新しいデータがない
    return false;
  }
  a_ahrs.mpu6050.dmpGetQuaternion(&a_ahrs.q, a_ahrs.fifo_buffer);
  a_ahrs.mpu6050.dmpGetGravity(&a_ahrs.gravity, &a_ahrs.q);
  a_ahrs.mpu6050.dmpGetYawPitchRoll(a_ahrs.ypr, &a_ahrs.q, &a_ahrs.gravity);
  a_ahrs.mpu6050.dmpGetAccel(&a_ahrs.aa, a_ahrs.fifo_buffer);
  a_ahrs.mpu6050.dmpGetGyro(&a_ahrs.gyro, a_ahrs.fifo_buffer);

  const float rad2deg_tmp = 180.0f / float(M_PI);  // float で計算する(M_PI は double)
  a_ahrs.yaw_source = a_ahrs.ypr[0] * rad2deg_tmp; // ソースデータ保持
  a_ahrs.roll_source = a_ahrs.ypr[2] * rad2deg_tmp;
  a_ahrs.pitch_source = a_ahrs.ypr[1] * rad2deg_tmp;

  // BNO055と同じ単位にそろえる. DMPの加速度は 1g = 8192, ジャイロは ±2000deg/s(16.4 = 1deg/s)
  const float accel_k_tmp = 9.80665f / 8192.0f; // → m/s^2
  const float gyro_k_tmp = 1.0f / 16.4f;        // → deg/s

  float val_tmp[16];
  val_tmp[0] = a_ahrs.aa.x * accel_k_tmp; // 加速度(m/s^2)
  val_tmp[1] = a_ahrs.aa.y * accel_k_tmp;
  val_tmp[2] = a_ahrs.aa.z * accel_k_tmp;
  val_tmp[3] = a_ahrs.gyro.x * gyro_k_tmp; // ジャイロ(deg/s)
  val_tmp[4] = a_ahrs.gyro.y * gyro_k_tmp;
  val_tmp[5] = a_ahrs.gyro.z * gyro_k_tmp;
  val_tmp[6] = 0; // 磁力センサ(MPU6050にはない)
  val_tmp[7] = 0;
  val_tmp[8] = 0;
  val_tmp[9] = a_ahrs.gravity.x; // DMPによる重力方向の推定値
  val_tmp[10] = a_ahrs.gravity.y;
  val_tmp[11] = a_ahrs.gravity.z;
  val_tmp[12] = mrd_wire0_deg_from_origin(a_ahrs.roll_source, a_ahrs.roll_origin);   // DMP_ROLL推定値
  val_tmp[13] = mrd_wire0_deg_from_origin(a_ahrs.pitch_source, a_ahrs.pitch_origin); // DMP_PITCH推定値
  val_tmp[14] = mrd_wire0_deg_from_origin(a_ahrs.yaw_source, a_ahrs.yaw_origin);     // DMP_YAW推定値
  val_tmp[15] = a_ahrs.mpu6050.getTemperature() / 340.0f + 36.53f;                   // 温度(degC. MPU-6050 のデータシートの換算式)

  portENTER_CRITICAL(&ahrs_mux);
  memcpy(a_ahrs.read, val_tmp, sizeof(val_tmp));
  portEXIT_CRITICAL(&ahrs_mux);
  return true;
}

/// @brief MPU6050からI2C経由でデータを読み取るスレッド用関数. IMUAHRS_INTERVALの間隔で実行する.
///        新しいデータが 500ms 以上届かなければ, 応答なしとして表示する(値は前回のまま).
void mrd_wire0_mpu6050_task(void *a_args) {
  bool ok_prev_tmp = true;
  unsigned long last_ok_ms_tmp = millis();
  while (1) {
    if (mrd_wire0_mpu6050_read(ahrs)) {
      last_ok_ms_tmp = millis();
    }
    mrd_wire0_health_report("MPU6050", millis() - last_ok_ms_tmp < 500, ok_prev_tmp);
    delay(cfg.imuahrs_interval);
  }
}

//------------------------------------------------------------------------------------
//  meriput
//------------------------------------------------------------------------------------

/// @brief 指定されたIMU/AHRSタイプに基づいて, 計測したAHRSデータを読み込む.
/// @param a_meridim 書き込み先のMeridim配列.
/// @param a_type 使用するセンサのタイプを示す列挙(MPU6050_IMU, BNO055_AHRS. MPU9250は未対応).
/// @param a_ahrs_result AHRSから読み取った結果を格納した配列.
/// @return データの書き込みが成功した場合はtrue, それ以外の場合はfalseを返す.
bool mrd_meriput90_ahrs(Meridim90Union &a_meridim, float a_ahrs_result[], int a_type) {
  if (a_type == BNO055_AHRS || a_type == MPU6050_IMU) {
    // センサスレッドの書き込み途中の値を読まないよう, 排他区間でまとめてコピーしてから変換する
    float val_tmp[16];
    portENTER_CRITICAL(&ahrs_mux);
    memcpy(val_tmp, a_ahrs_result, sizeof(val_tmp));
    portEXIT_CRITICAL(&ahrs_mux);
    a_meridim.sval[2] = mrd.float2HfShort(val_tmp[0]);   // IMU/AHRS_acc_x
    a_meridim.sval[3] = mrd.float2HfShort(val_tmp[1]);   // IMU/AHRS_acc_y
    a_meridim.sval[4] = mrd.float2HfShort(val_tmp[2]);   // IMU/AHRS_acc_z
    a_meridim.sval[5] = mrd.float2HfShort(val_tmp[3]);   // IMU/AHRS_gyro_x
    a_meridim.sval[6] = mrd.float2HfShort(val_tmp[4]);   // IMU/AHRS_gyro_y
    a_meridim.sval[7] = mrd.float2HfShort(val_tmp[5]);   // IMU/AHRS_gyro_z
    a_meridim.sval[8] = mrd.float2HfShort(val_tmp[6]);   // IMU/AHRS_mag_x
    a_meridim.sval[9] = mrd.float2HfShort(val_tmp[7]);   // IMU/AHRS_mag_y
    a_meridim.sval[10] = mrd.float2HfShort(val_tmp[8]);  // IMU/AHRS_mag_z
    a_meridim.sval[11] = mrd.float2HfShort(val_tmp[15]); // temperature
    a_meridim.sval[12] = mrd.float2HfShort(val_tmp[12]); // DMP_ROLL推定値
    a_meridim.sval[13] = mrd.float2HfShort(val_tmp[13]); // DMP_PITCH推定値
    a_meridim.sval[14] = mrd.float2HfShort(val_tmp[14]); // DMP_YAW推定値
    return true;
  }
  return false;
}

#endif // __MERIDIAN_WIRE0_H__
