#ifndef __MERIDIAN_SERVO_KONDO_ICS_H__
#define __MERIDIAN_SERVO_KONDO_ICS_H__

#include "config.h"
#include "main.h"

//================================================================================================================
//  KONDO ICSサーボ関連の処理
//  -----------------------------------------------------------------------------------
//================================================================================================================

/// @brief icsサーボについて, meridim配列に従ってサーボを目標位置に移動し, 角度を受信する.
/// UART_L,UART_Rに対し, 交互に1つずつサーボコマンドを送受信する.
/// @param a_meridim サーボの動作パラメータを含むMeridim配列.
/// @param a_sv サーボパラメータの構造体.
/// @return なし.
void mrd_sv_drive_ics_double(Meridim90Union a_meridim, ServoParam &a_sv) {
  for (int i = 0; i <= a_sv.num_max; i++) // 接続したサーボの数だけ繰り返す. 最大は15
  {
    int k = 0;
    if (a_sv.idl_mount[i] == 43) // ICS_L系統の処理
    {
      if (r_udp_meridim.sval[(i * 2) + 20] == 1) // 受信配列のサーボコマンドが1ならPos指定
      {
        k = ics_L.setPos(i, mrd.Deg2Krs(a_sv.idl_tgt[i], a_sv.idl_trim[i], a_sv.idl_cw[i]));
        if (k == -1) // サーボからの返信信号を受け取れなかった時は前回の数値のままにする
        {
          k = mrd.Deg2Krs(a_sv.idl_tgt_past[i], a_sv.idl_trim[i], a_sv.idl_cw[i]);
          if (a_sv.idl_err[i] < 100) {
            a_sv.idl_err[i]++;
          }
          if (a_sv.idl_err[i] >= SERVO_LOST_ERROR_WAIT) {
            s_udp_meridim.ubval[MRD_ERR_l] = min(
                uint8_t(i + 100),
                uint8_t(
                    149)); // Meridim[MRD_ERR] エラーを出したサーボID（100をID[L00]として[L49]まで）
            mrd_msg_servo_err(L, i, monitor.servo_err);
          }
        } else {
          a_sv.idl_err[i] = 0;
        }
      } else // 1以外ならとりあえずサーボを脱力し位置を取得.
      {
        k = ics_L.setFree(i); // サーボからの返信信号を受け取れていれば値を更新
        if (k == -1) // サーボからの返信信号を受け取れなかった時は前回の数値のままにする
        {
          k = mrd.Deg2Krs(a_sv.idl_tgt_past[i], a_sv.idl_trim[i], a_sv.idl_cw[i]);
          a_sv.idl_err[i]++;
          // エラーが出たサーボID（100をID[L00]として[L49]まで）
          if (a_sv.idl_err[i] >= SERVO_LOST_ERROR_WAIT) {
            s_udp_meridim.ubval[MRD_ERR_l] = min(uint8_t(i + 100), uint8_t(149));
            mrd_msg_servo_err(L, i, monitor.servo_err);
          }
        } else {
          a_sv.idl_err[i] = 0;
        }
      }
      a_sv.idl_tgt[i] = mrd.Krs2Deg(k, a_sv.idl_trim[i], a_sv.idl_cw[i]);
    }
    delayMicroseconds(2);

    if (a_sv.idr_mount[i] == 43) // ICS_R系統の処理
    {
      if (r_udp_meridim.sval[(i * 2) + 50] == 1) // 受信配列のサーボコマンドが1ならPos指定
      {
        k = ics_R.setPos(i, mrd.Deg2Krs(a_sv.idr_tgt[i], a_sv.idr_trim[i], a_sv.idr_cw[i]));
        if (k == -1) // サーボからの返信信号を受け取れなかった時は前回の数値のままにする
        {
          k = mrd.Deg2Krs(a_sv.idr_tgt_past[i], a_sv.idr_trim[i], a_sv.idr_cw[i]);
          if (a_sv.idr_err[i] < 10) {
            a_sv.idr_err[i]++;
          }
          if (a_sv.idr_err[i] >= SERVO_LOST_ERROR_WAIT) {
            // エラーが出たサーボID（200をID[R00]として[R49]まで）
            s_udp_meridim.ubval[MRD_ERR_l] = min(uint8_t(i + 200), uint8_t(249));
            mrd_msg_servo_err(R, i, monitor.servo_err);
          }
        } else {
          a_sv.idr_err[i] = 0;
        }
      } else // 1以外ならとりあえずサーボを脱力し位置を取得
      {
        k = ics_R.setFree(i);
        if (k == -1) // サーボからの返信信号を受け取れなかった時は前回の数値のままにする
        {
          k = mrd.Deg2Krs(a_sv.idr_tgt_past[i], a_sv.idr_trim[i], a_sv.idr_cw[i]);
          a_sv.idr_err[i]++;
          if (a_sv.idr_err[i] >= SERVO_LOST_ERROR_WAIT) {
            s_udp_meridim.ubval[MRD_ERR_l] = min(
                uint8_t(i + 200),
                uint8_t(
                    249)); // Meridim[MRD_ERR] エラーを出したサーボID（200をID[R00]として[R49]まで）
            mrd_msg_servo_err(R, i, monitor.servo_err);
          }
        } else {
          a_sv.idr_err[i] = 0;
        }
      }
      a_sv.idr_tgt[i] = mrd.Krs2Deg(k, a_sv.idr_trim[i], a_sv.idr_cw[i]);
    }
    delayMicroseconds(2);
  }
}

/*
bool flg_dual_follow = false;
void mrd_sv_drive_ics_dual(Meridim90Union a_meridim)
{
    flg_dual_follow = true;

    for (int i = 0; i < a_sv.num_max; i++) // ICS_L系統の処理
    {                                    // 接続したサーボの数だけ繰り返す. 最大は15
        int k = 0;
        if (a_sv.idl_mount[i])
        {
            if (r_udp_meridim.sval[(i * 2) + 20] == 1) // 受信配列のサーボコマンドが1ならPos指定
            {
                k = ics_L.setPos(i, mrd.Deg2Krs(a_sv.idl_tgt[i], a_sv.idl_trim[i], a_sv.idl_cw[i]));
                if (k == -1) // サーボからの返信信号を受け取れなかった時は前回の数値のままにする
                {
                    k = mrd.Deg2Krs(a_sv.idl_tgt_past[i], a_sv.idl_trim[i], a_sv.idl_cw[i]);
                    if (a_sv.idl_err[i] < 100)
                    {
                        a_sv.idl_err[i]++;
                    }
                    if (a_sv.idl_err[i] >= SERVO_LOST_ERROR_WAIT)
                    {
                        s_udp_meridim.ubval[MRD_ERR_l] = min(uint8_t(i + 100), uint8_t(149)); //
Meridim[MRD_ERR] エラーを出したサーボID（100をID[L00]として[L49]まで） mrd_msg_servo_err(L, i,
monitor.servo_err);
                    }
                }
                else
                {
                    a_sv.idl_err[i] = 0;
                }
            }
            else // 1以外ならとりあえずサーボを脱力し位置を取得. 手持ちの最大は15
            {
                k = ics_L.setFree(i); // サーボからの返信信号を受け取れていれば値を更新
                if (k == -1)          //
サーボからの返信信号を受け取れなかった時は前回の数値のままにする
                {
                    k = mrd.Deg2Krs(a_sv.idl_tgt_past[i], a_sv.idl_trim[i], a_sv.idl_cw[i]);
                    a_sv.idl_err[i]++;
                    if (a_sv.idl_err[i] >= SERVO_LOST_ERROR_WAIT)
                    {
                        s_udp_meridim.ubval[MRD_ERR_l] = min(uint8_t(i + 100), uint8_t(149)); //
Meridim[MRD_ERR] エラーを出したサーボID（100をID[L00]として[L49]まで） mrd_msg_servo_err(L, i,
monitor.servo_err);
                    }
                }
                else
                {
                    a_sv.idl_err[i] = 0;
                }
            }
            a_sv.idl_tgt[i] = mrd.Krs2Deg(k, a_sv.idl_trim[i], a_sv.idl_cw[i]);
        }
        delayMicroseconds(2);
    }
}

void Core1_servo_dual(void *args)
{
    while (1)
    {

        while (!flg_dual_follow)
        {
            delay(1);
        }

        for (int i = 0; i < a_sv.num_max; i++) // ICS_L系統の処理
        {
            int k = 0;
            if (a_sv.idr_mount[i])
            {
                if (r_udp_meridim.sval[(i * 2) + 50] == 1) // 受信配列のサーボコマンドが1ならPos指定
                {
                    k = ics_R.setPos(i, mrd.Deg2Krs(a_sv.idr_tgt[i], a_sv.idr_trim[i],
a_sv.idr_cw[i])); if (k == -1) // サーボからの返信信号を受け取れなかった時は前回の数値のままにする
                    {
                        k = mrd.Deg2Krs(a_sv.idr_tgt_past[i], a_sv.idr_trim[i], a_sv.idr_cw[i]);
                        if (a_sv.idr_err[i] < 10)
                        {
                            a_sv.idr_err[i]++;
                        }
                        if (a_sv.idr_err[i] >= SERVO_LOST_ERROR_WAIT)
                        {
                            s_udp_meridim.ubval[MRD_ERR_l] = min(uint8_t(i + 200), uint8_t(249)); //
Meridim[MRD_ERR] エラーを出したサーボID（200をID[R00]として[R49]まで） mrd_msg_servo_err(R, i,
monitor.servo_err);
                        }
                    }
                    else
                    {
                        a_sv.idr_err[i] = 0;
                    }
                }
                else // 1以外ならとりあえずサーボを脱力し位置を取得
                {
                    k = ics_R.setFree(i);
                    if (k == -1) // サーボからの返信信号を受け取れなかった時は前回の数値のままにする
                    {
                        k = mrd.Deg2Krs(a_sv.idr_tgt_past[i], a_sv.idr_trim[i], a_sv.idr_cw[i]);
                        a_sv.idr_err[i]++;
                        if (a_sv.idr_err[i] >= SERVO_LOST_ERROR_WAIT)
                        {
                            s_udp_meridim.ubval[MRD_ERR_l] = min(uint8_t(i + 200), uint8_t(249)); //
Meridim[MRD_ERR] エラーを出したサーボID（200をID[R00]として[R49]まで） mrd_msg_servo_err(R, i,
monitor.servo_err);
                        }
                    }
                    else
                    {
                        a_sv.idr_err[i] = 0;
                    }
                }
                a_sv.idr_tgt[i] = mrd.Krs2Deg(k, a_sv.idr_trim[i], a_sv.idr_cw[i]);
            }
            delayMicroseconds(2);
        }
        flg_dual_follow = false;
    }
}
*/

#endif //__MERIDIAN_SERVO_KONDO_ICS_H__
