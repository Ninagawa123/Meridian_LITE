#ifndef __MERIDIAN_COMMAND_H__
#define __MERIDIAN_COMMAND_H__

// ヘッダファイルの読み込み
#include "config.h"
#include "mrd_common.h"
#include <Arduino.h> // HardwareSerial用
// 注意: main.h は MPU6050 の多重定義エラーを避けるため mrd_command.cpp でインクルード
// 注意: mrd_eeprom.h, mrd_servo.h の関数は mrd_command.cpp で extern 宣言して使用

//==================================================================================================
//  コマンド関数宣言
//==================================================================================================

/// @brief マスターコマンドグループ1を実行する. 受信コマンドに応じて異なる処理を行う.
/// @param a_meridim 実行するコマンドを含むMeridim配列 (参照渡し)
/// @param a_flg_exe Meridim受信成功フラグ
/// @param a_sv サーボパラメータ構造体 (参照渡し)
/// @param a_serial 出力シリアル
/// @return コマンドが実行された場合はtrue, 実行されなかった場合はfalse
bool execute_master_command_1(Meridim90Union &a_meridim, bool a_flg_exe, ServoParam &a_sv, HardwareSerial &a_serial);

/// @brief マスターコマンドグループ2を実行する. 受信コマンドに応じて異なる処理を行う.
/// @param a_meridim 実行するコマンドを含むMeridim配列 (参照渡し)
/// @param a_flg_exe Meridim受信成功フラグ
/// @param a_sv サーボパラメータ構造体 (参照渡し)
/// @param a_serial 出力シリアル
/// @return コマンドが実行された場合はtrue, 実行されなかった場合はfalse
bool execute_master_command_2(Meridim90Union &a_meridim, bool a_flg_exe, ServoParam &a_sv, HardwareSerial &a_serial);

/// @brief マスターコマンドグループ3を実行する. 受信コマンドに応じて異なる処理を行う.
/// @param a_meridim 実行するコマンドを含むMeridim配列 (参照渡し)
/// @param a_flg_exe Meridim受信成功フラグ
/// @param a_sv サーボパラメータ構造体 (参照渡し)
/// @param a_serial 出力シリアル
/// @return コマンドが実行された場合はtrue, 実行されなかった場合はfalse
bool execute_master_command_3(Meridim90Union &a_meridim, bool a_flg_exe, ServoParam &a_sv, HardwareSerial &a_serial);

#endif // __MERIDIAN_COMMAND_H__
