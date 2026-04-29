#ifndef __MERIDIAN_COMMAND_H__
#define __MERIDIAN_COMMAND_H__

// ヘッダファイルの読み込み
#include "mrd_common.h"

// ライブラリ導入
#include <IcsHardSerialClass.h>
#include <Meridian.h> // Meridianのライブラリ導入

//==================================================================================================
//  コマンド関数宣言
//==================================================================================================

/// @brief マスターコマンドグループ1を実行する. 受信コマンドに応じて異なる処理を行う.
/// @param a_meridim 実行するコマンドを含むMeridim配列 (参照渡し)
/// @param a_flg_exe Meridim受信成功フラグ
/// @param a_sv サーボパラメータ構造体 (参照渡し)
/// @param a_serial 出力シリアル
/// @param a_ics_L L系統のICS通信クラス (参照渡し)
/// @param a_ics_R R系統のICS通信クラス (参照渡し)
/// @param a_flg 各種フラグ構造体 (参照渡し)
/// @param a_mrd Meridianクラス (参照渡し)
/// @return コマンドが実行された場合はtrue, 実行されなかった場合はfalse
bool execute_master_command_1(Meridim90Union &a_meridim, bool a_flg_exe,
                              ServoParam &a_sv, HardwareSerial &a_serial,
                              IcsHardSerialClass &a_ics_L, IcsHardSerialClass &a_ics_R,
                              MrdFlags &a_flg, MERIDIANFLOW::Meridian &a_mrd);

/// @brief マスターコマンドグループ2を実行する. 受信コマンドに応じて異なる処理を行う.
/// @param a_meridim 実行するコマンドを含むMeridim配列 (参照渡し)
/// @param a_flg_exe Meridim受信成功フラグ
/// @param a_s_udp_meridim 送信用Meridim配列 (参照渡し)
/// @param a_sv サーボパラメータ構造体 (参照渡し)
/// @param a_serial 出力シリアル
/// @param a_flg 各種フラグ構造体 (参照渡し)
/// @return コマンドが実行された場合はtrue, 実行されなかった場合はfalse
bool execute_master_command_2(Meridim90Union &a_meridim, bool a_flg_exe,
                              Meridim90Union &a_s_udp_meridim, ServoParam &a_sv,
                              HardwareSerial &a_serial, MrdFlags &a_flg);

/// @brief マスターコマンドグループ3を実行する. 受信コマンドに応じて異なる処理を行う.
/// @param a_meridim 実行するコマンドを含むMeridim配列 (参照渡し)
/// @param a_flg_exe Meridim受信成功フラグ
/// @param a_sv サーボパラメータ構造体 (参照渡し)
/// @param a_serial 出力シリアル
/// @param a_ics_L L系統のICS通信クラス (参照渡し)
/// @param a_ics_R R系統のICS通信クラス (参照渡し)
/// @param a_flg 各種フラグ構造体 (参照渡し)
/// @param a_mrd Meridianクラス (参照渡し)
/// @return コマンドが実行された場合はtrue, 実行されなかった場合はfalse
bool execute_master_command_3(Meridim90Union &a_meridim, bool a_flg_exe, ServoParam &a_sv,
                              HardwareSerial &a_serial,
                              IcsHardSerialClass &a_ics_L, IcsHardSerialClass &a_ics_R,
                              MrdFlags &a_flg, MERIDIANFLOW::Meridian &a_mrd);

#endif // __MERIDIAN_COMMAND_H__
