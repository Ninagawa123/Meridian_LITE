#ifndef __MERIDIAN_SERVO_DISTRIBUTOR_H__
#define __MERIDIAN_SERVO_DISTRIBUTOR_H__

// ヘッダファイルの読み込み
#include "config.h"
#include "mrd_common.h"
#include <IcsHardSerialClass.h>
#include <Meridian.h>

// サーボドライバーの条件付きインクルード
// 使用するドライバーのみコンパイルに含める
#if MOUNT_SERVO_TYPE_L == KOICS3 || MOUNT_SERVO_TYPE_R == KOICS3
#include "mrd_module/sv_ics.h"
#endif

#if MOUNT_SERVO_TYPE_L == FTBRSX || MOUNT_SERVO_TYPE_R == FTBRSX
#include "mrd_module/sv_ftbrx.h"
#endif

#if MOUNT_SERVO_TYPE_L == DXL2 || MOUNT_SERVO_TYPE_R == DXL2
#include "mrd_module/sv_dxl2.h"
#endif

#if MOUNT_SERVO_TYPE_L == FTCSTS || MOUNT_SERVO_TYPE_R == FTCSTS || \
    MOUNT_SERVO_TYPE_L == FTCSCS || MOUNT_SERVO_TYPE_R == FTCSCS
#include "mrd_module/sv_ftc.h"
#endif

// グローバル変数のextern宣言 (実体はmain.cppで定義)
extern MERIDIANFLOW::Meridian mrd;
extern IcsHardSerialClass ics_L;
extern IcsHardSerialClass ics_R;

//==================================================================================================
//  サーボ関数宣言
//==================================================================================================

//------------------------------------------------------------------------------------
//  各UARTの開始
//------------------------------------------------------------------------------------

/// @brief 指定されたUARTラインとサーボタイプに基づいてサーボ通信プロトコルを設定する
/// @param a_line UART通信ライン (L, R, または C)
/// @param a_servo_type サーボのタイプを示す整数値
/// @return サーボがサポートされている場合はtrue, サポートされていない場合はfalse
bool mrd_servo_begin(UartLine a_line, int a_servo_type);

//------------------------------------------------------------------------------------
//  サーボ通信形成の分岐
//------------------------------------------------------------------------------------

/// @brief 指定されたサーボへコマンドを配信する
/// @param a_meridim サーボ動作パラメータを含むMeridim配列
/// @param a_L_type L系統のサーボタイプ
/// @param a_R_type R系統のサーボタイプ
/// @param a_sv サーボパラメータ構造体 (参照渡し)
/// @return サーボ駆動が成功した場合はtrue, 失敗した場合はfalse
bool mrd_servo_drive_lite(Meridim90Union &a_meridim, int a_L_type, int a_R_type, ServoParam &a_sv);

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
