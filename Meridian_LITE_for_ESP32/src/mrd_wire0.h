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

/// @brief 指定したI2Cアドレスの機器が応答するかを調べる.
bool mrd_wire0_probe(uint8_t a_addr) {
  Wire.beginTransmission(a_addr);
  return Wire.endTransmission() == 0;
}

/// @brief I2Cバスで応答した機器のアドレスを表示する(配線確認用).
void mrd_wire0_scan() {
  Serial.print("I2C devices found:");
  int count = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    if (mrd_wire0_probe(addr)) {
      Serial.print(" 0x");
      Serial.print(addr, HEX);
      count++;
    }
  }
  Serial.println(count ? "" : " none");
}

/// @brief MPU6050/MPU9250のジャイロのゼロ点を, 静止時の平均値から補正する.
///        ライブラリのCalibrateGyro/CalibrateAccelは誤差が収まるまで無限に繰り返すため
///        (加速度はZ軸が真上で水平静止が前提), 回数を固定した補正で起動が止まらないようにする.
///        動かしていると補正が不正確になるだけで, DMPの自動補正でも徐々に追従する.
/// @param a_mpu MPU6050のインスタンス(DMP初期化後, ジャイロレンジ±2000deg/s).
void mrd_wire0_calib_mpu_gyro(MPU6050 &a_mpu) {
  const int samples = 200;
  Serial.print("Calibrating MPU gyro (keep the sensor still)... ");
  for (int pass = 0; pass < 2; pass++) {
    long sum[3] = {0, 0, 0};
    for (int n = 0; n < samples; n++) {
      int16_t g[3];
      a_mpu.getRotation(&g[0], &g[1], &g[2]);
      for (int i = 0; i < 3; i++) {
        sum[i] += g[i];
      }
      delay(2);
    }
    // ±2000deg/sの1LSBはオフセットレジスタ(±1000deg/s単位)の2LSB
    a_mpu.setXGyroOffset(constrain(a_mpu.getXGyroOffset() - sum[0] * 2 / samples, -32768, 32767));
    a_mpu.setYGyroOffset(constrain(a_mpu.getYGyroOffset() - sum[1] * 2 / samples, -32768, 32767));
    a_mpu.setZGyroOffset(constrain(a_mpu.getZGyroOffset() - sum[2] * 2 / samples, -32768, 32767));
  }
  a_mpu.resetFIFO();
  a_mpu.resetDMP();
  Serial.println("done.");
}

/// @brief MPU6050センサーのDMP(デジタルモーションプロセッサ)を初期化し,
///        ジャイロスコープと加速度センサーのオフセットを設定する.
/// @param a_ahrs AHRSの値を保持する構造体.
/// @return DMPの初期化が成功した場合はtrue, 失敗した場合はfalseを返す.
bool mrd_wire0_init_mpu6050_dmp(AhrsValue &a_ahrs) {
  // センサが応答しないままライブラリを呼ぶと, 読み込みごとに1秒待って起動が数分止まるため先に確認する
  if (mrd_wire0_probe(MPU6050_ADDRESS_AD0_LOW)) {
    a_ahrs.mpu6050 = MPU6050(MPU6050_ADDRESS_AD0_LOW);
  } else if (mrd_wire0_probe(MPU6050_ADDRESS_AD0_HIGH)) {
    a_ahrs.mpu6050 = MPU6050(MPU6050_ADDRESS_AD0_HIGH);
  } else {
    Serial.println("No MPU6050/MPU9250 detected at 0x68/0x69 ... Check your wiring (SDA/SCL pins)!");
    mrd_wire0_scan();
    return false;
  }
  Serial.print("MPU found (WHO_AM_I 0x");
  Serial.print(a_ahrs.mpu6050.getDeviceID() << 1, HEX);
  Serial.println("). Initializing DMP...");

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
    mrd_wire0_calib_mpu_gyro(a_ahrs.mpu6050); // 加速度の傾きはMCMD_SENSOR_ALL_CALIB(Set Zero)で補正
    a_ahrs.mpu6050.setDMPEnabled(true);
    a_ahrs.packetSize = a_ahrs.mpu6050.dmpGetFIFOPacketSize();
    Serial.println("MPU6050 OK.");
    return true;
  }
  Serial.println("IMU/AHRS DMP Initialization FAILED!");
  return false;
}

// AK8963(MPU9250内蔵の磁気センサ)のレジスタ
#define AK8963_ADDR  0x0C // I2Cアドレス(MPU9250のI2Cバイパス経由)
#define AK8963_WIA   0x00 // デバイスID(0x48)
#define AK8963_ST1   0x02 // bit0:データレディ
#define AK8963_HXL   0x03 // 計測値 HXL〜HZH(リトルエンディアン), 続けてST2
#define AK8963_CNTL1 0x0A // 動作モード
#define AK8963_ASAX  0x10 // 感度補正値(Fuse ROM)

/// @brief AK8963のレジスタに1バイト書き込む.
bool mrd_wire0_ak8963_write(uint8_t a_reg, uint8_t a_data) {
  Wire.beginTransmission(AK8963_ADDR);
  Wire.write(a_reg);
  Wire.write(a_data);
  return Wire.endTransmission() == 0;
}

/// @brief AK8963のレジスタから連続して読み込む.
bool mrd_wire0_ak8963_read(uint8_t a_reg, uint8_t *a_data, uint8_t a_len) {
  Wire.beginTransmission(AK8963_ADDR);
  Wire.write(a_reg);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }
  if (Wire.requestFrom((uint8_t)AK8963_ADDR, a_len) != a_len) {
    return false;
  }
  for (int i = 0; i < a_len; i++) {
    a_data[i] = Wire.read();
  }
  return true;
}

/// @brief MPU9250センサーを初期化する. 加速度・ジャイロはMPU6050互換のためMPU6050のDMPを使い,
///        磁気センサAK8963はI2Cバイパスで直接読む.
/// @param a_ahrs AHRSの値を保持する構造体.
/// @return DMPの初期化が成功した場合はtrue(磁気センサが無くてもtrue), 失敗した場合はfalseを返す.
bool mrd_wire0_init_mpu9250_dmp(AhrsValue &a_ahrs) {
  if (!mrd_wire0_init_mpu6050_dmp(a_ahrs)) {
    return false;
  }
  // DMPの初期化でI2Cマスタは無効になっているので, バイパスでAK8963をESP32のI2Cにつなぐ
  a_ahrs.mpu6050.setI2CMasterModeEnabled(false); // バイパスはI2Cマスタ無効時のみ有効
  a_ahrs.mpu6050.setI2CBypassEnabled(true);
  delay(10);
  uint8_t data[3];
  bool found = false;
  for (int retry = 0; retry < 5 && !found; retry++) {
    found = mrd_wire0_ak8963_read(AK8963_WIA, data, 1) && data[0] == 0x48;
    if (!found) {
      delay(20);
    }
  }
  if (!found) {
    // MPU9250として売られている基板には磁気センサの無いMPU6500(WHO_AM_I 0x70)のものがある
    Serial.println("No AK8963 (MPU9250 magnetometer) detected. mag_x,y,z will be 0.");
    Serial.print("I2C master: ");
    Serial.print(a_ahrs.mpu6050.getI2CMasterModeEnabled() ? "on" : "off");
    Serial.print(", bypass: ");
    Serial.println(a_ahrs.mpu6050.getI2CBypassEnabled() ? "on" : "off");
    mrd_wire0_scan(); // マスタoff, バイパスonで0x0Cが無ければ磁気センサ非搭載
    return true;
  }
  // 感度補正値の読み込み(Fuse ROMアクセスモード)
  mrd_wire0_ak8963_write(AK8963_CNTL1, 0x00);
  delay(10);
  mrd_wire0_ak8963_write(AK8963_CNTL1, 0x0F);
  delay(10);
  if (!mrd_wire0_ak8963_read(AK8963_ASAX, data, 3)) {
    data[0] = data[1] = data[2] = 128; // 補正なし
  }
  for (int i = 0; i < 3; i++) {
    // 16bit出力の感度 0.15uT/LSB に補正値を掛けておく
    a_ahrs.mag_scale[i] = 0.15f * (((float)data[i] - 128) / 256.0f + 1.0f);
  }
  mrd_wire0_ak8963_write(AK8963_CNTL1, 0x00);
  delay(10);
  mrd_wire0_ak8963_write(AK8963_CNTL1, 0x16); // 16bit出力, 連続測定モード2(100Hz)
  delay(10);
  a_ahrs.mag_available = true;
  Serial.println("AK8963 (MPU9250 magnetometer) mounted.");
  return true;
}

/// @brief BNO055センサーの初期化を試みます.
/// @param a_ahrs AHRSの値を保持する構造体.
/// @return BNO055センサーの初期化が成功した場合はtrue, それ以外の場合はfalseを返す.
///         現在, この関数は常にfalseを返すように設定されています.
bool mrd_wire0_init_bno055(AhrsValue &a_ahrs) {
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
  // データの取得はセンサー用スレッドで実行
}

/// @brief 指定されたIMU/AHRSタイプに応じて適切なセンサの初期化を行います.
/// @param a_imuahrs_type 使用するセンサのタイプを示す列挙型です(MPU6050, MPU9250, BNO055).
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
    return mrd_wire0_init_mpu9250_dmp(a_ahrs);
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

/// @brief 角度を-180以上180未満に収める.
/// @param a_deg 角度(度)
/// @return -180 <= 戻り値 < 180 の角度
float mrd_wire0_wrap180(float a_deg) {
  while (a_deg >= 180) {
    a_deg -= 360;
  }
  while (a_deg < -180) {
    a_deg += 360;
  }
  return a_deg;
}

/// @brief bno055からI2C経由でデータを読み取るスレッド用関数. IMUAHRS_INTERVAL の間隔で実行する.
void mrd_wire0_Core0_bno055_r(void *args) {
  while (1) {
    // 加速度センサ値の取得と表示 - VECTOR_ACCELEROMETER - m/s^2
    imu::Vector<3> accelerometer = ahrs.bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    ahrs.read[0] = (float)accelerometer.x();
    ahrs.read[1] = (float)accelerometer.y();
    ahrs.read[2] = (float)accelerometer.z();

    // ジャイロセンサ値の取得 - VECTOR_GYROSCOPE - deg/s (Adafruit_BNO055: 1dps = 16 LSB)
    imu::Vector<3> gyroscope = ahrs.bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    ahrs.read[3] = gyroscope.x();
    ahrs.read[4] = gyroscope.y();
    ahrs.read[5] = gyroscope.z();

    // 磁力センサ値の取得と表示  - VECTOR_MAGNETOMETER - uT
    imu::Vector<3> magnetometer = ahrs.bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
    ahrs.read[6] = magnetometer.x();
    ahrs.read[7] = magnetometer.y();
    ahrs.read[8] = magnetometer.z();

    // センサフュージョンによる方向推定値の取得と表示 - VECTOR_EULER - degrees
    imu::Vector<3> euler = ahrs.bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    ahrs.roll_source = euler.y();                                         // ロール軸のソースデータ保持
    ahrs.pitch_source = euler.z();                                        // ピッチ軸のソースデータ保持
    ahrs.yaw_source = euler.x();                                          // ヨー軸のソースデータ保持
    ahrs.read[12] = mrd_wire0_wrap180(euler.y() - ahrs.roll_origin);  // DMP_ROLL推定値
    ahrs.read[13] = mrd_wire0_wrap180(euler.z() - ahrs.pitch_origin); // DMP_PITCH推定値
    ahrs.read[14] = mrd_wire0_wrap180(euler.x() - ahrs.yaw_origin);   // DMP_YAW推定値
    // 温度の取得 - degC
    ahrs.read[15] = (float)ahrs.bno.getTemp();

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

/// @brief AK8963の計測値をMPU9250の加速度・ジャイロと同じ座標軸に揃えてa_ahrs.readに格納する.
///        AK8963のX,YはMPU9250のY,Xに対応し, Zは逆向き.
void mrd_wire0_read_ak8963(AhrsValue &a_ahrs) {
  uint8_t data[7];
  if (!a_ahrs.mag_available) {
    return;
  }
  if (!mrd_wire0_ak8963_read(AK8963_ST1, data, 1) || !(data[0] & 0x01)) {
    return; // 新しいデータなし
  }
  if (!mrd_wire0_ak8963_read(AK8963_HXL, data, 7)) { // ST2まで読むと次の計測に進む
    return;
  }
  if (data[6] & 0x08) {
    return; // 磁気センサのオーバーフロー
  }
  float mx = (int16_t)(data[1] << 8 | data[0]) * a_ahrs.mag_scale[0];
  float my = (int16_t)(data[3] << 8 | data[2]) * a_ahrs.mag_scale[1];
  float mz = (int16_t)(data[5] << 8 | data[4]) * a_ahrs.mag_scale[2];
  a_ahrs.read[6] = my;  // mag_x - uT
  a_ahrs.read[7] = mx;  // mag_y - uT
  a_ahrs.read[8] = -mz; // mag_z - uT
}

/// @brief AHRSセンサーからI2C経由でデータを読み取る関数.
/// MPU6050, MPU9250(加速度・ジャイロはMPU6050互換, 磁気はAK8963)に対応.
/// 各データはBNO055と同じ単位(m/s^2, deg/s, uT, degC)で`ahrs.read`配列に格納され,
/// 利用可能な場合は`ahrs.result`にコピーされる.
bool mrd_wire0_read_ahrs_i2c(AhrsValue &a_ahrs) {

  if (MOUNT_IMUAHRS == MPU6050_IMU || MOUNT_IMUAHRS == MPU9250_IMU) { // MPU6050, MPU9250
    if (a_ahrs.mpu6050.dmpGetCurrentFIFOPacket(a_ahrs.fifoBuffer)) {  // Get new data
      a_ahrs.mpu6050.dmpGetQuaternion(&a_ahrs.q, a_ahrs.fifoBuffer);
      a_ahrs.mpu6050.dmpGetGravity(&a_ahrs.gravity, &a_ahrs.q);
      a_ahrs.mpu6050.dmpGetYawPitchRoll(a_ahrs.ypr, &a_ahrs.q, &a_ahrs.gravity);

      // acceleration values - m/s^2 (DMP FIFO: 1g = 8192 LSB)
      a_ahrs.mpu6050.dmpGetAccel(&a_ahrs.aa, a_ahrs.fifoBuffer);
      a_ahrs.read[0] = a_ahrs.aa.x * (9.80665f / 8192.0f);
      a_ahrs.read[1] = a_ahrs.aa.y * (9.80665f / 8192.0f);
      a_ahrs.read[2] = a_ahrs.aa.z * (9.80665f / 8192.0f);

      // gyro values - deg/s (DMP gyro range +/-2000deg/s: 1dps = 16.4 LSB)
      a_ahrs.mpu6050.dmpGetGyro(&a_ahrs.gyro, a_ahrs.fifoBuffer);
      a_ahrs.read[3] = a_ahrs.gyro.x / 16.4f;
      a_ahrs.read[4] = a_ahrs.gyro.y / 16.4f;
      a_ahrs.read[5] = a_ahrs.gyro.z / 16.4f;

      // magnetic field values - uT (MPU6050は磁気センサなしのため0)
      if (MOUNT_IMUAHRS == MPU9250_IMU) {
        mrd_wire0_read_ak8963(a_ahrs);
      } else {
        a_ahrs.read[6] = 0;
        a_ahrs.read[7] = 0;
        a_ahrs.read[8] = 0;
      }

      // Estimated gravity DMP value.
      a_ahrs.read[9] = a_ahrs.gravity.x;
      a_ahrs.read[10] = a_ahrs.gravity.y;
      a_ahrs.read[11] = a_ahrs.gravity.z;

      // Estimated heading value using DMP.
      // ソース値を保持し, MCMD_SENSOR_YAW_CALIB / MCMD_SENSOR_ALL_CALIB の原点を差し引く
      a_ahrs.roll_source = a_ahrs.ypr[2] * 180 / M_PI;
      a_ahrs.pitch_source = a_ahrs.ypr[1] * 180 / M_PI;
      a_ahrs.yaw_source = a_ahrs.ypr[0] * 180 / M_PI;
      a_ahrs.read[12] = mrd_wire0_wrap180(a_ahrs.roll_source - a_ahrs.roll_origin);   // Estimated DMP_ROLL
      a_ahrs.read[13] = mrd_wire0_wrap180(a_ahrs.pitch_source - a_ahrs.pitch_origin); // Estimated DMP_PITCH
      a_ahrs.read[14] = mrd_wire0_wrap180(a_ahrs.yaw_source - a_ahrs.yaw_origin);     // Estimated DMP_YAW

      // Temperature - degC
      if (MOUNT_IMUAHRS == MPU9250_IMU) {
        a_ahrs.read[15] = a_ahrs.mpu6050.getTemperature() / 333.87f + 21.0f;
      } else {
        a_ahrs.read[15] = a_ahrs.mpu6050.getTemperature() / 340.0f + 36.53f;
      }

      if (flg.imuahrs_available) {
        memcpy(a_ahrs.result, a_ahrs.read, sizeof(float) * 16);
      }
      return true;
    } else {
      return false;
    }
  }
  return false;
}

/// @brief MPU6050, MPU9250からI2C経由でデータを読み取るスレッド用関数. IMUAHRS_INTERVAL の間隔で実行する.
void mrd_wire0_Core0_mpu_r(void *args) {
  while (1) {
    mrd_wire0_read_ahrs_i2c(ahrs);
    delay(IMUAHRS_INTERVAL);
  }
}

//------------------------------------------------------------------------------------
//  meriput
//------------------------------------------------------------------------------------

/// @brief 指定されたIMU/AHRSタイプに基づいて, 計測したAHRSデータを読み込む.
/// @param a_type 使用するセンサのタイプを示す列挙(MPU6050, MPU9250, BNO055).
/// @param a_ahrs_result AHRSから読み取った結果を格納した配列.
/// @return データの書き込みが成功した場合はtrue, それ以外の場合はfalseを返す.
bool meriput90_ahrs(Meridim90Union &a_meridim, float a_ahrs_result[], int a_type) {
  if (a_type == BNO055_AHRS || a_type == MPU6050_IMU || a_type == MPU9250_IMU) {
    flg.imuahrs_available = false;
    a_meridim.sval[2] = mrd.float2HfShort(a_ahrs_result[0]);   // IMU/AHRS_acc_x
    a_meridim.sval[3] = mrd.float2HfShort(a_ahrs_result[1]);   // IMU/AHRS_acc_y
    a_meridim.sval[4] = mrd.float2HfShort(a_ahrs_result[2]);   // IMU/AHRS_acc_z
    a_meridim.sval[5] = mrd.float2HfShort(a_ahrs_result[3]);   // IMU/AHRS_gyro_x
    a_meridim.sval[6] = mrd.float2HfShort(a_ahrs_result[4]);   // IMU/AHRS_gyro_y
    a_meridim.sval[7] = mrd.float2HfShort(a_ahrs_result[5]);   // IMU/AHRS_gyro_z
    a_meridim.sval[8] = mrd.float2HfShort(a_ahrs_result[6]);   // IMU/AHRS_mag_x
    a_meridim.sval[9] = mrd.float2HfShort(a_ahrs_result[7]);   // IMU/AHRS_mag_y
    a_meridim.sval[10] = mrd.float2HfShort(a_ahrs_result[8]);  // IMU/AHRS_mag_z
    a_meridim.sval[11] = mrd.float2HfShort(a_ahrs_result[15]); // temperature
    a_meridim.sval[12] = mrd.float2HfShort(a_ahrs_result[12]); // DMP_ROLL推定値
    a_meridim.sval[13] = mrd.float2HfShort(a_ahrs_result[13]); // DMP_PITCH推定値
    a_meridim.sval[14] = mrd.float2HfShort(a_ahrs_result[14]); // DMP_YAW推定値
    flg.imuahrs_available = true;
    return true;
  }
  return false;
}

#endif // __MERIDIAN_WIRE0_H__
