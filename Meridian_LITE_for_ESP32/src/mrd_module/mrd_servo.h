#ifndef __MERIDIAN_SERVO_DISTRIBUTOR_H__
#define __MERIDIAN_SERVO_DISTRIBUTOR_H__

// ヘッダファイルの読み込み
#include "../config.h"
#include "mrd_types.h"
#include <IcsHardSerialClass.h>
#include <Arduino.h>

// ics_L, ics_R は mrd_types.h で extern 宣言済み

#include "sv_ftbrx.h"
#include "sv_ics.h"

//==================================================================================================
//  関数プロトタイプ宣言
//==================================================================================================

//------------------------------------------------------------------------------------
//  各UARTの開始
//------------------------------------------------------------------------------------

/// @brief 指定されたUARTラインとサーボタイプに基づいてサーボの通信プロトコルを設定する.
/// @param a_line UART通信ライン(L, R, またはC).
/// @param a_servo_type サーボのタイプを示す整数値.
/// @return サーボがサポートされている場合はtrueを, サポートされていない場合はfalseを返す.
bool mrd_servo_begin(UartLine a_line, int a_servo_type);

//------------------------------------------------------------------------------------
//  サーボ通信フォーメーションの分岐
//------------------------------------------------------------------------------------

/// @brief 指定されたサーボにコマンドを分配する.
/// @param a_meridim サーボの動作パラメータを含むMeridim配列.
/// @param a_L_type L系統のサーボタイプ.
/// @param a_R_type R系統のサーボタイプ.
/// @param a_sv サーボパラメータの構造体.
/// @return サーボの駆動が成功した場合はtrueを, 失敗した場合はfalseを返す.
bool mrd_servos_drive_lite(Meridim90Union &a_meridim, int a_L_type, int a_R_type, ServoParam &a_sv);

//------------------------------------------------------------------------------------
//  各種オペレーション
//------------------------------------------------------------------------------------

/// @brief 第一引数のMeridim配列のすべてのサーボモーターをオフ(フリー状態)に設定する.
/// @param a_meridim サーボの動作パラメータを含むMeridim配列.
/// @return 設定完了時にtrueを返す.
bool mrd_servo_all_off(Meridim90Union &a_meridim);

/// @brief サーボパラメータからエラーのあるサーボのインデックス番号を作る.
/// @param a_sv サーボパラメータの構造体.
/// @return uint8_tで番号を返す.
///         100-149(L系統 0-49),200-249(R系統 0-49)
uint8_t mrd_servos_make_errcode_lite(ServoParam a_sv);

#endif // __MERIDIAN_SERVO_DISTRIBUTOR_H__
