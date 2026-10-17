#ifndef __MERIDIAN_COMMAND_H__
#define __MERIDIAN_COMMAND_H__

// ヘッダファイルの読み込み
#include "config.h"
#include "main.h"

// ライブラリ導入
#include "mrd_eeprom.h"
#include "mrd_servo.h"

//==================================================================================================
//  コマンド処理
//==================================================================================================

/// @brief コマンドの実行をシリアルに表示する. 例) "cmd: reset servo error id.[10004]"
/// @param a_text 内容.
/// @param a_code コマンドの番号.
void mrd_command_log(HardwareSerial &a_serial, const String &a_text, int a_code) {
  a_serial.println("cmd: " + a_text + ".[" + String(a_code) + "]");
}

/// @brief EEPROM の P1 からトリムと回転方向を読み込み, 全サーボを原点(目標値0)へ動かす(10100, 10102).
/// @param a_keep_past 動かす前の値を前回の値(tgt_past)として保持するか.
void mrd_command_trim_home(Meridim90Union &a_meridim, ServoParam &a_sv, bool a_keep_past, HardwareSerial &a_serial) {
  mrd_eeprom_servosettings_load(a_sv, true, a_serial);
  for (int i = 0; i < MRD_SERVO_SLOTS; i++) {
    a_meridim.sval[MRD_L_ORIGIDX + 1 + i * 2] = 0; // L系統の目標値を原点に
    a_meridim.sval[MRD_R_ORIGIDX + 1 + i * 2] = 0; // R系統の目標値を原点に
  }
  for (int i = 0; i < IXL_MAX; i++) {
    if (a_keep_past) {
      a_sv.ixl_tgt_past[i] = a_sv.ixl_tgt[i];
    }
    a_sv.ixl_tgt[i] = 0;
  }
  for (int i = 0; i < IXR_MAX; i++) {
    if (a_keep_past) {
      a_sv.ixr_tgt_past[i] = a_sv.ixr_tgt[i];
    }
    a_sv.ixr_tgt[i] = 0;
  }
  if (!cfg.mode_esp32_standalone) {
    mrd_servo_drive_lite(a_meridim, cfg.mount_servo_type_l, cfg.mount_servo_type_r, a_sv);
  }
}

/// @brief Master Commandの第1群を実行する. 受信コマンドに基づき, 異なる処理を行う.
/// @param a_meridim 実行したいコマンドの入ったMeridim配列.(参照渡し)
/// @param a_flg_exe Meridimの受信成功判定フラグ.
/// @param a_sv サーボパラメータの構造体.(参照渡し)
/// @return コマンドを実行した場合はtrue, しなかった場合はfalseを返す.
bool mrd_command_group1_execute(Meridim90Union &a_meridim, bool a_flg_exe, ServoParam &a_sv, HardwareSerial &a_serial) {
  if (!a_flg_exe) {
    return false;
  }
  // コマンド[90]: 1~999は MeridimのLength. デフォルトは90

  // コマンド:MCMD_ERR_CLEAR_SERVO_ID (10004) 通信エラーサーボIDのクリア
  if (a_meridim.sval[MRD_MASTER] == MCMD_ERR_CLEAR_SERVO_ID) {
    a_meridim.bval[MRD_ERR_l] = 0;
    for (int i = 0; i < IXL_MAX; i++) { // 失敗回数と通信不能の記録を消す. 応答しないままなら再び数え直す
      a_sv.ixl_err[i] = 0;
      a_sv.ixl_stat[i] = 0;
    }
    for (int i = 0; i < IXR_MAX; i++) {
      a_sv.ixr_err[i] = 0;
      a_sv.ixr_stat[i] = 0;
    }
    mrd_command_log(a_serial, "reset servo error id", MCMD_ERR_CLEAR_SERVO_ID);
    return true;
  }

  // コマンド:MCMD_BOARD_TRANSMIT_ACTIVE (10005) UDP受信の通信周期制御をボード側主導に(デフォルト). LITEではフラグとフレームタイマーのリセットのみ
  if (a_meridim.sval[MRD_MASTER] == MCMD_BOARD_TRANSMIT_ACTIVE) {
    flg.udp_board_passive = false; // UDP送信をアクティブモードに
    flg.count_frame_reset = true;  // フレームの管理時計をリセットフラグをセット
    return true;
  }

  // コマンド:MCMD_EEPROM_ENTER_WRITE (10009) EEPROMの書き込みモードスタート. LITEではフラグとフレームタイマーのリセットのみ(書き込みは10300+p)
  if (a_meridim.sval[MRD_MASTER] == MCMD_EEPROM_ENTER_WRITE) {
    flg.eeprom_write_mode = true; // 書き込みモードのフラグをセット
    flg.count_frame_reset = true; // フレームの管理時計をリセットフラグをセット
    return true;
  }

  // コマンド:MCMD_EEPROM_SAVE_TRIM (10101) 受信したスロットの設定ワードとトリムを EEPROM の P1 に書く(Meridian_console 連携)
  //         マウント・IDは次の起動から, トリム・回転方向は 10100 / 10102 で反映する
  if (a_meridim.sval[MRD_MASTER] == MCMD_EEPROM_SAVE_TRIM) {
    mrd_command_log(a_serial, "set EEPROM data from current trim", MCMD_EEPROM_SAVE_TRIM);

    // P1 の形: スロットの語([20]〜[79])だけを使い, num_max 以降のスロットと他の語は0
    Meridim90Union page_tmp = Meridim90Union();
    for (int i = 0; i < a_sv.num_max; i++) {
      page_tmp.sval[MRD_L_ORIGIDX + i * 2] = a_meridim.sval[MRD_L_ORIGIDX + i * 2];
      page_tmp.sval[MRD_L_ORIGIDX + 1 + i * 2] = a_meridim.sval[MRD_L_ORIGIDX + 1 + i * 2];
      page_tmp.sval[MRD_R_ORIGIDX + i * 2] = a_meridim.sval[MRD_R_ORIGIDX + i * 2];
      page_tmp.sval[MRD_R_ORIGIDX + 1 + i * 2] = a_meridim.sval[MRD_R_ORIGIDX + 1 + i * 2];
    }
    if (mrd_eeprom_page_store(MRD_EEP_P_SERVO, page_tmp, EEPROM_PROTECT, a_serial)) {
      a_serial.println("write EEPROM succeed.");
    } else {
      a_serial.println("write EEPROM failed.");
      return false;
    }
    return true;
  }

  // コマンド:MCMD_EEPROM_PCTOBOARD_DATA0 + p (10300〜10315) 受信した [2]〜[87] を EEPROM のページ p に書く.
  //         P1 のトリム・回転方向はすぐ反映し, それ以外は次の起動から反映する. 返信は 10300+p, 失敗なら MCMD_NAK
  if (a_meridim.sval[MRD_MASTER] >= MCMD_EEPROM_PCTOBOARD_DATA0 &&
      a_meridim.sval[MRD_MASTER] < MCMD_EEPROM_PCTOBOARD_DATA0 + MRD_EEP_PAGES) {
    const int page_tmp = a_meridim.sval[MRD_MASTER] - MCMD_EEPROM_PCTOBOARD_DATA0;
    const bool ok_tmp = mrd_eeprom_page_store(page_tmp, a_meridim, EEPROM_PROTECT, a_serial);
    if (ok_tmp && page_tmp == MRD_EEP_P_SERVO) {
      mrd_eeprom_servosettings_load(a_sv, false, a_serial);
    }
    a_meridim.sval[MRD_MASTER] = ok_tmp ? short(MCMD_EEPROM_PCTOBOARD_DATA0 + page_tmp) : short(MCMD_NAK);
    mrd_command_log(a_serial, "write EEPROM page " + String(page_tmp) + (ok_tmp ? "" : " failed"), MCMD_EEPROM_PCTOBOARD_DATA0 + page_tmp);
    return ok_tmp;
  }

  // コマンド:MCMD_BOARD_RESTART (10400) ボードを再起動する(EEPROM に書いた設定を反映するため).
  //         動作中の誤送信で再起動しないよう, 全スロットのコマンドが0(脱力)のときだけ受け付ける
  if (a_meridim.sval[MRD_MASTER] == MCMD_BOARD_RESTART) {
    for (int i = 0; i < MRD_SERVO_SLOTS; i++) {
      if (mrd_servo_cmd(a_meridim.sval[MRD_L_ORIGIDX + i * 2]) != 0 || mrd_servo_cmd(a_meridim.sval[MRD_R_ORIGIDX + i * 2]) != 0) {
        mrd_command_log(a_serial, "restart refused (set all servo commands to 0)", MCMD_BOARD_RESTART);
        return false;
      }
    }
    mrd_command_log(a_serial, "restart the board", MCMD_BOARD_RESTART);
    delay(100); // 表示を送り終えるのを待つ
    ESP.restart();
    return true;
  }

  // コマンド:MCMD_EEPROM_LOAD_TRIM (10102) EEPROMからTRIM値を読み込んで設定
  if (a_meridim.sval[MRD_MASTER] == MCMD_EEPROM_LOAD_TRIM) {
    // EEPROMのTRIM値で補正されたHOME(原点)にサーボを移動する
    mrd_command_trim_home(a_meridim, a_sv, false, a_serial);
    flg.count_frame_reset = true; // フレームの管理時計をリセットフラグをセット
    return true;
  }

  return false;
}

/// @brief Master Commandの第2群を実行する. 受信コマンドに基づき, 異なる処理を行う.
/// @param a_meridim 実行したいコマンドの入ったMeridim配列.(参照渡し)
/// @param a_flg_exe Meridimの受信成功判定フラグ. コマンド0と2は, 脱力を確実にするためこのフラグを見ない.
/// @param a_sv サーボパラメータの構造体.(参照渡し)
/// @return コマンドを実行した場合はtrue, しなかった場合はfalseを返す.
bool mrd_command_group2_execute(Meridim90Union &a_meridim, bool a_flg_exe, ServoParam &a_sv, HardwareSerial &a_serial) {
  // コマンド[90]: 1~999は MeridimのLength. デフォルトは90

  // コマンド:[0] 全サーボ脱力(可能な場合は位置を読む). 受信できなかったフレーム(前回の値を使う)でも実行する
  if (a_meridim.sval[MRD_MASTER] == MCMD_TORQUE_ALL_OFF) {
    mrd_servo_all_off(a_meridim, a_flg_exe); // 表示は正しく受信したフレームだけ(途絶えている間は出さない)
    return true;
  }

  // コマンド:[1] サーボオン 通常動作

  // コマンド:[2] 全サーボ脱力(位置は読まない. 返信は前回の値). 0と同じく受信の成否を見ない
  if (a_meridim.sval[MRD_MASTER] == 2) {
    mrd_servo_all_off(a_meridim, a_flg_exe);
    mrd_servo_skip_reads_frame = true;
    return true;
  }

  if (!a_flg_exe) {
    return false;
  }

  // コマンド:[3] 全サーボコマンド受付(ESP32内の計算を優先). 目標値の扱いは main.cpp [7-1]

  // コマンド:MCMD_SENSOR_YAW_CALIB (10002) IMU/AHRSのヨー軸リセット
  if (a_meridim.sval[MRD_MASTER] == MCMD_SENSOR_YAW_CALIB) {
    ahrs.yaw_origin = ahrs.yaw_source;
    mrd_command_log(a_serial, "caliblate sensor's yaw", MCMD_SENSOR_YAW_CALIB);
    return true;
  }

  // コマンド:MCMD_SENSOR_ALL_CALIB (10003) IMU/AHRSのロール・ピッチ・ヨーの現在値を補正センターにする(現在値が0になる)
  if (a_meridim.sval[MRD_MASTER] == MCMD_SENSOR_ALL_CALIB) {
    ahrs.roll_origin = ahrs.roll_source;
    ahrs.pitch_origin = ahrs.pitch_source;
    ahrs.yaw_origin = ahrs.yaw_source;
    mrd_command_log(a_serial, "caliblate sensor's roll, pitch and yaw", MCMD_SENSOR_ALL_CALIB);
    return true;
  }

  // コマンド:MCMD_BOARD_TRANSMIT_PASSIVE (10006) UDP受信の通信周期制御をPC側主導に(SSH的な動作). LITEではフラグとフレームタイマーのリセットのみ
  if (a_meridim.sval[MRD_MASTER] == MCMD_BOARD_TRANSMIT_PASSIVE) {
    flg.udp_board_passive = true; // UDP送信をパッシブモードに
    flg.count_frame_reset = true; // フレームの管理時計をリセットフラグをセット
    mrd_command_log(a_serial, "enter passive mode", MCMD_BOARD_TRANSMIT_PASSIVE);
    return true;
  }

  // コマンド:MCMD_FRAMETIMER_RESET (10007) フレームカウンタを現在時刻にリセット
  if (a_meridim.sval[MRD_MASTER] == MCMD_FRAMETIMER_RESET) {
    flg.count_frame_reset = true; // フレームの管理時計をリセットフラグをセット
    return true;
  }

  // コマンド:MCMD_BOARD_STOP_DURING (10008) ボードの末端処理を指定時間だけ止める.
  //         止めている間は受信もフェイルセーフも働かないため, 1000ms までにする
  if (a_meridim.sval[MRD_MASTER] == MCMD_BOARD_STOP_DURING) {
    const int stop_ms_tmp = constrain(int(a_meridim.sval[MRD_STOP_FRAMES]), 0, 1000);
    mrd_command_log(a_serial, "stop ESP32's processing during " + String(stop_ms_tmp) + " ms", MCMD_BOARD_STOP_DURING);

    if (stop_ms_tmp > 0) {
      delay(stop_ms_tmp);
    }
    flg.count_frame_reset = true; // フレームの管理時計をリセットフラグをセット
    return true;
  }
  return false;
}

/// @brief Master Commandの第3群を実行する. 受信コマンドに基づき, 異なる処理を行う.
/// @param a_meridim 実行したいコマンドの入ったMeridim配列.(参照渡し)
/// @param a_flg_exe Meridimの受信成功判定フラグ.
/// @param a_sv サーボパラメータの構造体.(参照渡し)
/// @return コマンドを実行した場合はtrue, しなかった場合はfalseを返す.
bool mrd_command_group3_execute(Meridim90Union &a_meridim, bool a_flg_exe, ServoParam &a_sv, HardwareSerial &a_serial) {
  if (!a_flg_exe) {
    return false;
  }
  // コマンド[90]: 1~999は MeridimのLength. デフォルトは90

  // コマンド:[0] 全サーボ脱力 は第2群で実行済み

  // コマンド:[1] サーボオン 通常動作

  // コマンド:MCMD_START_TRIM_SETTING (10100) TRIM設定のスタート(Meridian_console連携)
  if (a_meridim.sval[MRD_MASTER] == MCMD_START_TRIM_SETTING) {
    // EEPROMのTRIM値で補正されたHOME(原点)にサーボを移動する
    mrd_command_trim_home(a_meridim, a_sv, true, a_serial);

    // サーボの目標値として現在のTRIM値(degree ×100)をセットする
    for (int i = 0; i < MRD_SERVO_SLOTS; i++) {
      a_meridim.sval[MRD_L_ORIGIDX + 1 + i * 2] = (i < IXL_MAX) ? mrd.float2HfShort(a_sv.ixl_trim[i]) : 0;
      a_meridim.sval[MRD_R_ORIGIDX + 1 + i * 2] = (i < IXR_MAX) ? mrd.float2HfShort(a_sv.ixr_trim[i]) : 0;
    }

    // サーボのTRIM値をゼロリセットし, 同じ角度を保つよう目標を元のTRIM値の角度(trim×cw)にする.
    // 次のフレームの移動量制限の基準(cmd_past)も同じ値にする
    for (int i = 0; i < IXL_MAX; i++) {
      a_sv.ixl_tgt_past[i] = a_sv.ixl_tgt[i]; // 前回のdegreeをキープ
      a_sv.ixl_tgt[i] = a_sv.ixl_trim[i] * a_sv.ixl_cw[i];
      a_sv.ixl_cmd_past[i] = a_sv.ixl_tgt[i];
      a_sv.ixl_trim[i] = 0;
    }
    for (int i = 0; i < IXR_MAX; i++) {
      a_sv.ixr_tgt_past[i] = a_sv.ixr_tgt[i];
      a_sv.ixr_tgt[i] = a_sv.ixr_trim[i] * a_sv.ixr_cw[i];
      a_sv.ixr_cmd_past[i] = a_sv.ixr_tgt[i];
      a_sv.ixr_trim[i] = 0;
    }

    // サーボ動作を実行する. サーボはTRIM値を0としつつ, tgtとしてこれまでのTRIM値の角度をキープする
    if (!cfg.mode_esp32_standalone) {
      mrd_servo_drive_lite(a_meridim, cfg.mount_servo_type_l, cfg.mount_servo_type_r, a_sv); // サーボ動作を実行する
    }

    // サーボの設定値とTRIM値(EEPROM の P1)をPCに送信する
    mrd_eeprom_page_to_meridim(MRD_EEP_P_SERVO, a_meridim);
    a_meridim.sval[MRD_MASTER] = MCMD_EEPROM_BOARDTOPC_DATA1;

    a_serial.println("send:");
    for (int i = 0; i < MRDM_LEN; i++) {
      a_serial.print(a_meridim.sval[i]);
      a_serial.print(",");
    }
    a_serial.println();

    mrd_command_log(a_serial, "enter trim setting mode and send EEPROM[1][*] to PC", MCMD_START_TRIM_SETTING);
    return true;
  }

  // コマンド:MCMD_EEPROM_BOARDTOPC_DATA0 + p (10200〜10215) EEPROM のページ p を [2]〜[87] に入れて PC に送る.
  //         返信の [0] は 10200+p
  if (a_meridim.sval[MRD_MASTER] >= MCMD_EEPROM_BOARDTOPC_DATA0 &&
      a_meridim.sval[MRD_MASTER] < MCMD_EEPROM_BOARDTOPC_DATA0 + MRD_EEP_PAGES) {
    const int page_tmp = a_meridim.sval[MRD_MASTER] - MCMD_EEPROM_BOARDTOPC_DATA0;
    mrd_eeprom_page_to_meridim(page_tmp, a_meridim);

    mrd_command_log(a_serial, "send EEPROM page " + String(page_tmp) + " to PC", a_meridim.sval[MRD_MASTER]);
    return true;
  }

  return false;
}

#endif // __MERIDIAN_COMMAND_H__