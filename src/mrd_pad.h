#ifndef __MERIDIAN_JOYPAD_H__
#define __MERIDIAN_JOYPAD_H__

// ヘッダファイルの読み込み
#include "config.h"
#include "main.h"
#include "mrd_servo.h"

//================================================================================================================
//  JOYPAD 関連の処理
//================================================================================================================

//------------------------------------------------------------------------------------
//  各種パッドからの読み取り処理
//------------------------------------------------------------------------------------

/// @brief KRC-5FHジョイパッドからデータを読み取り, 指定された間隔でデータを更新する.
/// @param a_pre_val 前回のジョイパッドの状態を保持する64ビット整数.
/// @param a_interval 読み取り間隔（ミリ秒）.
/// @param a_marge マージフラグ. trueの場合, 古いデータと新しいデータをマージする.
/// @rcvd_tmpurn 更新されたジョイパッドの状態を64ビット整数で返す.
uint64_t mrd_pad_read_krc(uint64_t a_pre_val, uint a_interval, bool a_marge) {
  // KRR5FH(KRC-5FH)をICS_R系に接続している場合
  static unsigned long last_time_tmp = 0; // 最後に関数が呼ばれた時間を記録
  unsigned long current_time_tmp = millis();

  if (current_time_tmp - last_time_tmp >= a_interval) {
    static bool first_call_tmp = true; // 初回の呼び出しフラグ
    unsigned short krr_button_tmp;     // krrからのボタン入力データ
    int krr_analog_tmp[4];             // krrからのアナログ入力データ
    unsigned short pad_common_tmp = 0; // PS準拠に変換後のボタンデータ
    bool rcvd_tmp;                     // 受信機がデータを受信成功したか
    rcvd_tmp = ics_R.getKrrAllData(&krr_button_tmp, krr_analog_tmp);
    delayMicroseconds(2);
    if (rcvd_tmp) // ボタンデータが受信できていたら

    {
      int button_tmp = krr_button_tmp; // 受信ボタンデータの読み込み用

      if (PAD_GENERALIZE) {

        if ((button_tmp & 15) == 15) { // 左側十字ボタン全部押しなら start押下とみなす
          pad_common_tmp += 1;
        } else {
          // 左側の十字ボタン
          pad_common_tmp += (button_tmp & 1) * 16 + ((button_tmp & 2) >> 1) * 64 +
                            ((button_tmp & 4) >> 2) * 32 + ((button_tmp & 8) >> 3) * 128;
        }
        if ((button_tmp & 368) == 368)
          pad_common_tmp += 8; // 右側十字ボタン全部押しなら select押下とみなす
        else {
          // 右側十字ボタン
          pad_common_tmp += ((button_tmp & 16) >> 4) * 4096 + ((button_tmp & 32) >> 5) * 16384 +
                            ((button_tmp & 64) >> 6) * 8192 + ((button_tmp & 256) >> 8) * 32768;
        }
        // L1,L2,R1,R2
        pad_common_tmp += ((button_tmp & 2048) >> 11) * 2048 + ((button_tmp & 4096) >> 12) * 512 +
                          ((button_tmp & 512) >> 9) * 1024 + ((button_tmp & 1024) >> 10) * 256;

        int8_t pad_analog_tmp[4];
        if (krr_analog_tmp[0] + krr_analog_tmp[1] + krr_analog_tmp[2] + krr_analog_tmp[3]) {
          for (int i = 0; i < 4; i++) {
            pad_analog_tmp[i] = (krr_analog_tmp[i] - 62) << 2;
            pad_analog_tmp[i] = (pad_analog_tmp[i] < -127) ? -127 : pad_analog_tmp[i];
            pad_analog_tmp[i] = (pad_analog_tmp[i] > 127) ? 127 : pad_analog_tmp[i];
          }
        } else
          for (int i = 0; i < 4; i++) {
            pad_analog_tmp[i] = 0;
          }
      } else {
        pad_common_tmp = button_tmp;
      }

      if (first_call_tmp) {
        Serial.println("KRC-5FH successfully connected. ");
        first_call_tmp = false; // 初回の呼び出しフラグをオフにする
      }
    }
    // 共用体用の64ビットの上位16ビット部をボタンデータとして書き換える
    uint64_t updated_val_tmp;
    if (a_marge) {
      updated_val_tmp = (a_pre_val & 0xFFFF000000000000) |
                        (static_cast<uint64_t>(pad_common_tmp)); // 上位16ビット index[0]
    } else {
      updated_val_tmp = (a_pre_val) | (static_cast<uint64_t>(pad_common_tmp));
    }
    // updated_val_tmp = (updated_val_tmp & 0x0000FFFFFFFFFFFF) |
    // (static_cast<uint64_t>(pad_common_tmp) << 48); // 下位16ビット index[3] updated_val_tmp =
    // (updated_val_tmp & 0xFFFF0000FFFFFFFF) | (static_cast<uint64_t>(pad_common_tmp) << 32); //
    // 上位33-48ビット index[2] updated_val_tmp = (updated_val_tmp & 0xFFFFFFFF0000FFFF) |
    // (static_cast<uint64_t>(pad_common_tmp) << 16); // 上位17-32ビット index[1]
    last_time_tmp = current_time_tmp; // 最後の実行時間を更新
    return updated_val_tmp;
  }
  return a_pre_val;
}

//------------------------------------------------------------------------------------
//  各種パッド読み取りへの分岐
//------------------------------------------------------------------------------------

/// @brief 指定されたジョイパッドタイプに応じてデータを読み取り, 関数外のmeridim配列に代入する.
/// @param a_pad_type ジョイパッドのタイプを示す列挙型（MERIMOTE, BLUERETRO, SBDBT, KRR5FH）.
/// @param a_interval ジョイパッドのデータ読み取り間隔（ミリ秒）.
/// @param a_marge データをマージするかどうかのブール値.
/// trueの場合は既存のデータにビット単位でOR演算を行い, falseの場合は新しいデータで上書きする.
/// @param a_monitor データをシリアルモニタに表示するかどうか.
/// @return ジョイパッドの接続がプログラムされていないタイプの場合はfalseを返し, それ以外の場合はtrueを返す.
bool mrd_pad_reader(PadReceiverType a_pad_type, uint a_interval, bool a_marge, bool a_monitor) {
  if (a_pad_type == MERIMOTE) // merimote, 未対応.
  {
    for (int i = 0; i < 4; i++) {
      if (a_marge) {
        r_udp_meridim.sval[MRD_CONTROL_BUTTONS + i] |= pad_array.sval[i];
        s_udp_meridim.sval[MRD_CONTROL_BUTTONS + i] |= pad_array.sval[i];
      } else {
        r_udp_meridim.sval[MRD_CONTROL_BUTTONS + i] = pad_array.sval[i];
        s_udp_meridim.sval[MRD_CONTROL_BUTTONS + i] = pad_array.sval[i];
      }
    }
    if (a_monitor) {
      Serial.print("MERIMOTE:");
      mrd.monitor_joypad(pad_array.usval);
    }
  } else if (a_pad_type == BLUERETRO) // BLUERETRO, 未対応.
  {
    Serial.print("BLUERETRO connection has not been programmed yet.");
    return false;
  } else if (a_pad_type == SBDBT) // SBDBT, 未対応.
  {
    Serial.print("SBDBT connection has not been programmed yet.");
    return false;
  } else if (a_pad_type == KRR5FH) // KRC-5FH+KRR-5FH
  {
    pad_array.ui64val[0] = mrd_pad_read_krc(pad_array.ui64val[0], a_interval, a_marge);
    r_udp_meridim.sval[MRD_CONTROL_BUTTONS] |= pad_array.sval[0];
    s_udp_meridim.sval[MRD_CONTROL_BUTTONS] |= pad_array.sval[0];
    if (a_monitor) {
      Serial.print("KRR5FH:");
      mrd.monitor_joypad(pad_array.usval);
    }
  } else {
    pad_array.usval[0] = r_udp_meridim.sval[MRD_CONTROL_BUTTONS]; // Meridim受信値をセットする
    if (a_monitor) {
      Serial.print("No Remocon:");
      mrd.monitor_joypad(pad_array.usval);
    }
    return false;
  }

  return true;
}

//------------------------------------------------------------------------------------
//  各種オペレーション
//------------------------------------------------------------------------------------

#endif // __MERIDIAN_JOYPAD_H__
