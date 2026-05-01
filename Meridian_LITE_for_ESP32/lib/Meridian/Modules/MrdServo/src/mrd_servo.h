#ifndef __MERIDIAN_SERVO_DISTRIBUTOR_H__
#define __MERIDIAN_SERVO_DISTRIBUTOR_H__

// ヘッダファイルの読み込み
#include "mrd_common.h"

// ライブラリの読み込み
#include <IcsHardSerialClass.h>

//==================================================================================================
//  サーボ関数宣言
//==================================================================================================

//------------------------------------------------------------------------------------
//  各UARTの開始
//------------------------------------------------------------------------------------

/// @brief 指定されたUARTラインとサーボタイプに基づいてサーボ通信プロトコルを設定する
/// @param a_line UART通信ライン (L, R, または C)
/// @param a_servo_type サーボのタイプを示す整数値
/// @param a_ics サーボクラスのインスタンス (KONDO ICSサーボ用)
/// @return サーボがサポートされている場合はtrue, サポートされていない場合はfalse
bool mrd_servo_begin(UartLine a_line, ServoType a_servo_type, IcsHardSerialClass &a_ics);

//------------------------------------------------------------------------------------
//  サーボ通信形成の分岐
//------------------------------------------------------------------------------------

/// @brief 指定されたサーボへコマンドを配信する
/// @param a_meridim サーボ動作パラメータを含むMeridim配列
/// @param a_L_type L系統のサーボタイプ
/// @param a_R_type R系統のサーボタイプ
/// @param a_sv サーボパラメータ構造体 (参照渡し)
/// @param a_ics_L L系統のICSサーボ通信クラスのインスタンス
/// @param a_ics_R R系統のICSサーボ通信クラスのインスタンス
/// @param a_mrd Meridianクラスのインスタンス
/// @return サーボ駆動が成功した場合はtrue, 失敗した場合はfalse
bool mrd_servo_drive_lite(Meridim90Union &a_meridim, ServoType a_L_type, ServoType a_R_type,
                          ServoParam &a_sv,
                          IcsHardSerialClass &a_ics_L, IcsHardSerialClass &a_ics_R,
                          MERIDIANFLOW::Meridian &a_mrd);

//------------------------------------------------------------------------------------
//  各種オペレーション
//------------------------------------------------------------------------------------

/// @brief 第一引数のMeridim配列のすべてのサーボモーターをオフ(フリー状態)に設定する
/// @param a_meridim サーボ動作パラメータを含むMeridim配列
/// @return 完了時にtrueを返す
bool mrd_servo_all_off(Meridim90Union &a_meridim);

/// @brief サーボパラメータからエラーのあるサーボのインデックス番号を作成する
/// @param a_sv サーボパラメータ構造体 (参照渡し)
/// @return uint8_t の番号
///         100-149 (L系統 0-49), 200-249 (R系統 0-49)
uint8_t mrd_servo_make_errcode_lite(const ServoParam &a_sv);

#endif // __MERIDIAN_SERVO_DISTRIBUTOR_H__
