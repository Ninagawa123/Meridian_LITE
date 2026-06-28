#ifndef __MERIDIAN_COMMAND_H__
#define __MERIDIAN_COMMAND_H__

// ヘッダファイルの読み込み
#include "../config.h"
#include "mrd_types.h"
#include <Arduino.h>

// ライブラリ導入
#include "mrd_eeprom.h"
#include "mrd_servo.h"

//==================================================================================================
//  関数プロトタイプ宣言
//==================================================================================================

/// @brief Master Commandの第1群を実行する. 受信コマンドに基づき, 異なる処理を行う.
/// @param a_meridim 実行したいコマンドの入ったMeridim配列.(参照渡し)
/// @param a_flg_exe Meridimの受信成功判定フラグ.
/// @param a_sv サーボパラメータの構造体.(参照渡し)
/// @return コマンドを実行した場合はtrue, しなかった場合はfalseを返す.
bool execute_master_command_1(Meridim90Union &a_meridim, bool a_flg_exe, ServoParam &a_sv, HardwareSerial &a_serial);

/// @brief Master Commandの第2群を実行する. 受信コマンドに基づき, 異なる処理を行う.
/// @param a_meridim 実行したいコマンドの入ったMeridim配列.(参照渡し)
/// @param a_flg_exe Meridimの受信成功判定フラグ.
/// @param a_sv サーボパラメータの構造体.(参照渡し)
/// @return コマンドを実行した場合はtrue, しなかった場合はfalseを返す.
bool execute_master_command_2(Meridim90Union &a_meridim, bool a_flg_exe, ServoParam &a_sv, HardwareSerial &a_serial);

/// @brief Master Commandの第3群を実行する. 受信コマンドに基づき, 異なる処理を行う.
/// @param a_meridim 実行したいコマンドの入ったMeridim配列.(参照渡し)
/// @param a_flg_exe Meridimの受信成功判定フラグ.
/// @param a_sv サーボパラメータの構造体.(参照渡し)
/// @return コマンドを実行した場合はtrue, しなかった場合はfalseを返す.
bool execute_master_command_3(Meridim90Union &a_meridim, bool a_flg_exe, ServoParam &a_sv, HardwareSerial &a_serial);

#endif // __MERIDIAN_COMMAND_H__
