#ifndef __MERIDIAN_SERVO_KONDO_ICS_H__
#define __MERIDIAN_SERVO_KONDO_ICS_H__

#include "config.h"
#include "main.h"
#include "mrd_disp.h"
#include "mrd_servo.h" // 半二重UARTバス(MrdHalfDuplexUart). 本ファイルは mrd_servo.h から読み込まれる

#include "gs2d_krs.h"
#include <IcsBaseClass.h>

//==================================================================================================
//  KONDO ICS プロトコル層 (半二重UARTバス上で動作)
//==================================================================================================

/// @brief IcsBaseClassの送受信をMrdHalfDuplexUartに委譲するICSクラス.
///        setPos, setFree, getKrrAllData等はIcsBaseClassのものをそのまま使用する.
class IcsHalfDuplexClass : public IcsBaseClass {
public:
  /// @brief コンストラクタ.
  /// @param a_bus 使用する半二重UARTバス.
  /// @param a_baud ICSの通信速度(115200, 625000, 1250000bps).
  IcsHalfDuplexClass(MrdHalfDuplexUart &a_bus, unsigned long a_baud)
      : m_bus(a_bus), m_baud(a_baud) {}

  /// @brief ICS用のフレーム形式(8E1)でバスを開始する.
  bool begin() { return m_bus.begin(m_baud, SERIAL_8E1); }

  /// @brief IcsBaseClass の送受信処理. 半二重UARTバスの transfer() に委譲する.
  bool synchronize(byte *txBuf, byte txLen, byte *rxBuf, byte rxLen) override {
    return m_bus.transfer(txBuf, txLen, rxBuf, rxLen);
  }

private:
  MrdHalfDuplexUart &m_bus; // 使用する半二重UARTバス
  unsigned long m_baud;     // ICSの通信速度(bps)
};

//==================================================================================================
//  KONDO ICSサーボ関連の処理
//==================================================================================================

/// @brief ICSサーボの実行処理を行う関数
/// @param a_servo_id サーボのインデックス番号
/// @param a_cmd サーボのコマンド
/// @param a_tgt サーボの目標位置
/// @param a_tgt_past 前回のサーボの目標位置
/// @param a_trim サーボの補正値(degree)
/// @param a_cw サーボの回転方向補正値
/// @param a_err_cnt サーボのエラーカウント
/// @param a_stat サーボのステータス
/// @param ics サーボクラスのインスタンス
float mrd_servo_process_ics(int a_servo_id, int a_cmd, float a_tgt, float a_tgt_past, float a_trim,
                            int a_cw, int &a_err_cnt, uint16_t &a_stat, IcsBaseClass &ics) {
  int val_tmp = 0;
  if (a_cmd == 1) { // コマンドが1ならPos指定
    val_tmp = ics.setPos(a_servo_id, mrd.Deg2Krs(a_tgt, a_trim, a_cw));
  } else { // コマンドが0等なら脱力して値を取得
    val_tmp = ics.setFree(a_servo_id);
  }

  if (val_tmp == -1) { // サーボからの返信信号を受け取れなかった場合
    val_tmp = mrd.Deg2Krs(a_tgt_past, a_trim, a_cw);
    a_err_cnt++;
    if (a_err_cnt >= SERVO_LOST_ERR_WAIT) { // 一定以上の連続エラーで通信不能とみなす
      a_err_cnt = SERVO_LOST_ERR_WAIT;
      a_stat = 1;
    }
  } else {
    a_err_cnt = 0;
    a_stat = 0;
  }

  return mrd.Krs2Deg(val_tmp, a_trim, a_cw);
}

/// @brief ICSサーボを駆動する関数
/// @param a_meridim Meridimデータの参照
/// @param a_sv サーボパラメータの配列
void mrd_sv_drive_ics_double(Meridim90Union &a_meridim, ServoParam &a_sv) {
  for (int i = 0; i < a_sv.num_max; i++) {
    // L系統サーボの処理
    if (a_sv.ixl_mount[i]) { // 43は近藤科学のICSサーボ
      a_sv.ixl_tgt[i] = mrd_servo_process_ics(
          a_sv.ixl_id[i], a_meridim.sval[(i * 2) + 20], a_sv.ixl_tgt[i], a_sv.ixl_tgt_past[i],
          a_sv.ixl_trim[i], a_sv.ixl_cw[i], a_sv.ixl_err[i], a_sv.ixl_stat[i], ics_L);
    }
    // R系統サーボの処理
    if (a_sv.ixr_mount[i]) { // 43は近藤科学のICSサーボ
      a_sv.ixr_tgt[i] = mrd_servo_process_ics(
          a_sv.ixr_id[i], a_meridim.sval[(i * 2) + 50], a_sv.ixr_tgt[i], a_sv.ixr_tgt_past[i],
          a_sv.ixr_trim[i], a_sv.ixr_cw[i], a_sv.ixr_err[i], a_sv.ixr_stat[i], ics_R);
    }
    delayMicroseconds(2); // Teensyの場合には必要かも
  }
}

/// @brief ICSサーボを1系統だけ駆動する関数 (L/Rで異なる種類のサーボを使う場合)
/// @param a_meridim Meridimデータの参照
/// @param a_sv サーボパラメータの配列
/// @param a_line 駆動する系統
void mrd_sv_drive_ics_line(Meridim90Union &a_meridim, ServoParam &a_sv, UartLine a_line) {
  for (int i = 0; i < a_sv.num_max; i++) {
    if (a_line == L && a_sv.ixl_mount[i]) {
      a_sv.ixl_tgt[i] = mrd_servo_process_ics(
          a_sv.ixl_id[i], a_meridim.sval[(i * 2) + 20], a_sv.ixl_tgt[i], a_sv.ixl_tgt_past[i],
          a_sv.ixl_trim[i], a_sv.ixl_cw[i], a_sv.ixl_err[i], a_sv.ixl_stat[i], ics_L);
    } else if (a_line == R && a_sv.ixr_mount[i]) {
      a_sv.ixr_tgt[i] = mrd_servo_process_ics(
          a_sv.ixr_id[i], a_meridim.sval[(i * 2) + 50], a_sv.ixr_tgt[i], a_sv.ixr_tgt_past[i],
          a_sv.ixr_trim[i], a_sv.ixr_cw[i], a_sv.ixr_err[i], a_sv.ixr_stat[i], ics_R);
    }
  }
}

//==================================================================================================
//  gs2d によるICSサーボの処理
//  https://github.com/karakuri-products/gs2d
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
// float mrd_servo_process_ics_gs2d(int a_servo_id, int a_cmd,
//                                  float a_tgt, float a_tgt_past, int a_trim,
//                                  int a_cw, int &a_err_cnt, uint16_t &a_stat,
//                                  IcsHardSerialClass &ics) {
//   int val_tmp = 0;
//   if (a_cmd == 1) { // コマンドが1ならPos指定
//     val_tmp = ics.setPos(a_servo_id, mrd.Deg2Krs(a_tgt, a_trim, a_cw));
//   } else { // コマンドが0等なら脱力して値を取得
//     val_tmp = ics.setFree(a_servo_id);
//   }
//   if (val_tmp == -1) { // サーボからの返信信号を受け取れなかった場合
//     val_tmp = mrd.Deg2Krs(a_tgt_past, a_trim, a_cw);
//     a_err_cnt++;
//     if (a_err_cnt >= SERVO_LOST_ERR_WAIT) { // 一定以上の連続エラーで通信不能とみなす
//       a_err_cnt = SERVO_LOST_ERR_WAIT;
//       a_stat = 1;
//     }
//   } else {
//     a_err_cnt = 0;
//     a_stat = 0;
//   }

//   return mrd.Krs2Deg(val_tmp, a_trim, a_cw);
// }

#endif // __MERIDIAN_SERVO_KONDO_ICS_H__
