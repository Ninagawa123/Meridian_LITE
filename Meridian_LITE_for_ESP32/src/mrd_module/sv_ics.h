#ifndef __MERIDIAN_SERVO_KONDO_ICS_H__
#define __MERIDIAN_SERVO_KONDO_ICS_H__

#include "../config.h"
#include "mrd_types.h"
#include <IcsHardSerialClass.h>

// mrd, ics_L, ics_R は mrd_types.h で extern 宣言済み

//==================================================================================================
//  関数プロトタイプ宣言
//==================================================================================================

/// @brief ICSサーボの実行処理を行う関数
/// @param a_servo_id サーボのインデックス番号
/// @param a_cmd サーボのコマンド
/// @param a_tgt サーボの目標位置
/// @param a_tgt_past 前回のサーボの目標位置
/// @param a_tgt_trim サーボの補正値
/// @param a_cw サーボの回転方向補正値
/// @param a_err_cnt サーボのエラーカウント
/// @param a_stat サーボのステータス
/// @param ics サーボクラスのインスタンス
float mrd_sv_process_ics(int a_servo_id, int a_cmd, float a_tgt, float a_tgt_past, int a_trim,
                            int a_cw, int &a_err_cnt, uint16_t &a_stat, IcsHardSerialClass &ics);

/// @brief ICSサーボを駆動する関数
/// @param a_meridim Meridimデータの参照
/// @param a_sv サーボパラメータの配列
void mrd_sv_drive_ics_double(Meridim90Union &a_meridim, ServoParam &a_sv);

#endif // __MERIDIAN_SERVO_KONDO_ICS_H__
