#ifndef __MERIDIAN_WIRE0_H__
#define __MERIDIAN_WIRE0_H__

// ヘッダファイルの読み込み
#include "mrd_common.h"

// ライブラリ導入

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
/// @return DMP初期化が成功した場合はtrue, 失敗した場合はfalse
bool mrd_wire0_init_mpu6050_dmp();

/// @brief BNO055センサの初期化を試みる
/// @return BNO055初期化が成功した場合はtrue, 検出できなかった場合はfalse
bool mrd_wire0_init_bno055();

/// @brief 指定されたIMU/AHRSタイプに基づいて適切なセンサを初期化する
/// @param a_imuahrs_type センサタイプ (0:なし, 1:MPU6050, 2:MPU9250, 3:BNO055)
/// @param a_i2c0_speed I2C通信クロック速度
/// @param a_pinSDA SDAピン番号. a_pinSCLと共に省略可能. デフォルト -1.
/// @param a_pinSCL SCLピン番号. a_pinSDAと共に省略可能. デフォルト -1.
/// @return センサが正しく初期化された場合はtrue, それ以外はfalse
bool mrd_wire0_setup(ImuAhrsType a_imuahrs_type, int a_i2c0_speed, int a_pinSDA = -1, int a_pinSCL = -1);

//------------------------------------------------------------------------------------
//  センサデータ取得
//------------------------------------------------------------------------------------

/// @brief I2C経由でBNO055からデータを読み取るスレッド関数. IMUAHRS_INTERVAL間隔で実行.
/// @param args 未使用の引数
void mrd_wire0_Core0_bno055_r(void *args);

/// @brief I2C経由でAHRSセンサからデータを読み取る.
///        MPU6050, MPU9250用だが, MPU9250は未実装.
///        各データはahrs.read配列に格納され, 利用可能な場合ahrs.resultにコピーされる.
/// @param a_flg AHRSデータの更新フラグを含む構造体. 参照渡し.
/// @return 成功時はtrue, 失敗時はfalse
bool mrd_wire0_read_ahrs_i2c(MrdFlags &a_flg);

//------------------------------------------------------------------------------------
//  meriput
//------------------------------------------------------------------------------------

/// @brief 指定されたIMU/AHRSタイプに基づいて測定されたAHRSデータを読み取る
/// @param a_meridim Meridim配列共用体. 参照渡し.
/// @param a_ahrs_result AHRSから読み取った結果を格納する配列
/// @param a_type センサタイプのenum (MPU6050, MPU9250, BNO055)
/// @param mrd Meridianクラスのインスタンス. 参照渡し.
/// @return データ書き込みが成功した場合はtrue, それ以外はfalse
bool meriput90_ahrs(Meridim90Union &a_meridim, int a_type, MERIDIANFLOW::Meridian &mrd);

/// @brief AHRSセンサーのyaw_originを現在のyaw_sourceにキャリブレートする関数.
void mrd_wire0_calibrate_yaw_origin();

#endif // __MERIDIAN_WIRE0_H__
