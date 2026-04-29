#ifndef __MERIDIAN_WIRE0_H__
#define __MERIDIAN_WIRE0_H__

// ヘッダファイルの読み込み
#include "config.h"
#include "mrd_common.h"

// ライブラリ導入
#include <Wire.h>

// センサーライブラリの条件付きインクルード
#if MOUNT_IMUAHRS == BNO055_AHRS
#include <Adafruit_BNO055.h>
#endif
#if MOUNT_IMUAHRS == MPU6050_IMU
#include <MPU6050_6Axis_MotionApps20.h>
#endif

//------------------------------------------------------------------------------------
//  AhrsValue 構造体 (センサー固有の型を含む)
//------------------------------------------------------------------------------------

// 6軸or9軸センサーの値
struct AhrsValue {
#if MOUNT_IMUAHRS == BNO055_AHRS
  Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire); // BNO055のインスタンス
#endif

#if MOUNT_IMUAHRS == MPU6050_IMU
  MPU6050 mpu6050;        // MPU6050のインスタンス
  uint8_t mpuIntStatus;   // MPUからの割込みステータスバイト
  uint8_t devStatus;      // デバイス操作後の戻り値 (0:成功, 0以外:エラー)
  uint16_t packetSize;    // DMPパケットサイズ (デフォルト42バイト)
  uint8_t fifoBuffer[64]; // FIFOストレージバッファ
  Quaternion q;           // [w, x, y, z] クォータニオン格納用
  VectorFloat gravity;    // [x, y, z] 重力ベクトル
  VectorInt16 aa;         // [x, y, z] 加速度センサの測定値
  VectorInt16 gyro;       // [x, y, z] 角速度センサの測定値
  VectorInt16 mag;        // [x, y, z] 磁力センサの測定値
  long temperature;       // センサの温度測定値
#endif

  // 共通メンバ
  float ypr[3];         // [roll, pitch, yaw] ロール/ピッチ/ヨー格納用
  float yaw_origin = 0; // ヨー軸の補正センター値
  float yaw_source = 0; // ヨー軸のソースデータ保持用

  float read[16]; // mpuからの読み込んだ一次データacc_x,y,z,gyro_x,y,z,mag_x,y,z,gr_x,y,z,rpy_r,p,y,temp

  float zeros[16] = {0};               // リセット用
  float ave_data[16];                  // 上記の移動平均値を入れる
  float result[16];                    // 加工後の最新のmpuデータ(二次データ)
  float stock_data[IMUAHRS_STOCK][16]; // 上記の移動平均値計算用のデータストック
  int stock_count = 0;                 // 上記の移動平均値計算用のデータストックを輪番させる時の変数
};

// グローバル変数のextern宣言
extern AhrsValue ahrs;
extern SemaphoreHandle_t ahrs_mutex; // AHRSデータアクセス用mutex (main.cppで定義)

//==================================================================================================
//  I2C wire0 関数宣言
//==================================================================================================

//------------------------------------------------------------------------------------
//  初期化
//------------------------------------------------------------------------------------

/// @brief 指定されたクロック速度でWire0 I2C通信を初期化する
/// @param a_i2c0_speed I2C通信クロック速度
/// @param a_pinSDA SDAピン番号. a_pinSCLと共に省略可能. デフォルト -1.
/// @param a_pinSCL SCLピン番号. a_pinSDAと共に省略可能. デフォルト -1.
/// @return 完了時にtrueを返す
bool mrd_wire0_init_i2c(int a_i2c0_speed, int a_pinSDA = -1, int a_pinSCL = -1);

/// @brief MPU6050センサのDMP (Digital Motion Processor) を初期化し,
///        ジャイロスコープと加速度センサのオフセットを設定する
/// @param a_ahrs AHRS値を保持する構造体
/// @return DMP初期化が成功した場合はtrue, 失敗した場合はfalse
bool mrd_wire0_init_mpu6050_dmp(AhrsValue &a_ahrs);

/// @brief BNO055センサの初期化を試みる
/// @param a_ahrs AHRS値を保持する構造体
/// @return BNO055初期化が成功した場合はtrue, 検出できなかった場合はfalse
bool mrd_wire0_init_bno055(AhrsValue &a_ahrs);

/// @brief 指定されたIMU/AHRSタイプに基づいて適切なセンサを初期化する
/// @param a_imuahrs_type センサタイプのenum (MPU6050, MPU9250, BNO055)
/// @param a_i2c0_speed I2C通信クロック速度
/// @param a_ahrs AHRS値を保持する構造体
/// @param a_pinSDA SDAピン番号. a_pinSCLと共に省略可能. デフォルト -1.
/// @param a_pinSCL SCLピン番号. a_pinSDAと共に省略可能. デフォルト -1.
/// @return センサが正しく初期化された場合はtrue, それ以外はfalse
bool mrd_wire0_setup(ImuAhrsType a_imuahrs_type, int a_i2c0_speed, AhrsValue &a_ahrs,
                     int a_pinSDA = -1, int a_pinSCL = -1);

//------------------------------------------------------------------------------------
//  センサデータ取得
//------------------------------------------------------------------------------------

/// @brief I2C経由でBNO055からデータを読み取るスレッド関数. IMUAHRS_INTERVAL間隔で実行.
/// @param args 未使用の引数
void mrd_wire0_Core0_bno055_r(void *args);

/// @brief I2C経由でAHRSセンサからデータを読み取る.
///        MPU6050, MPU9250用だが, MPU9250は未実装.
///        各データはahrs.read配列に格納され, 利用可能な場合ahrs.resultにコピーされる.
/// @param a_ahrs AHRS値を保持する構造体
/// @return 成功時はtrue, 失敗時はfalse
bool mrd_wire0_read_ahrs_i2c(AhrsValue &a_ahrs);

//------------------------------------------------------------------------------------
//  meriput
//------------------------------------------------------------------------------------

/// @brief 指定されたIMU/AHRSタイプに基づいて測定されたAHRSデータを読み取る
/// @param a_meridim Meridim配列共用体. 参照渡し.
/// @param a_ahrs_result AHRSから読み取った結果を格納する配列
/// @param a_type センサタイプのenum (MPU6050, MPU9250, BNO055)
/// @return データ書き込みが成功した場合はtrue, それ以外はfalse
bool meriput90_ahrs(Meridim90Union &a_meridim, float a_ahrs_result[], int a_type);

#endif // __MERIDIAN_WIRE0_H__
