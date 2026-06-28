// mrd_command.cpp - Master command processing implementation

#include "mrd_command.h"

//==================================================================================================
//  コマンド処理
//==================================================================================================

/// @brief Master Commandの第1群を実行する. 受信コマンドに基づき, 異なる処理を行う.
/// @param a_meridim 実行したいコマンドの入ったMeridim配列.(参照渡し)
/// @param a_flg_exe Meridimの受信成功判定フラグ.
/// @param a_sv サーボパラメータの構造体.(参照渡し)
/// @return コマンドを実行した場合はtrue, しなかった場合はfalseを返す.
bool execute_master_command_1(Meridim90Union &a_meridim, bool a_flg_exe, ServoParam &a_sv, HardwareSerial &a_serial) {
  if (!a_flg_exe) {
    return false;
  }
  // コマンド[90]: 1~999は MeridimのLength. デフォルトは90

  // コマンド:MCMD_ERR_CLEAR_SERVO_ID (10004) 通信エラーサーボIDのクリア
  if (a_meridim.sval[MRD_MASTER] == MCMD_ERR_CLEAR_SERVO_ID) {
    a_meridim.bval[MRD_ERR_l] = 0;
    for (int i = 0; i < IXL_MAX; i++) {
      a_sv.ixl_err[i] = 0;
    }
    for (int i = 0; i < IXR_MAX; i++) {
      a_sv.ixr_err[i] = 0;
    }
    char tmp_msg[64];
    snprintf(tmp_msg, sizeof(tmp_msg), "cmd: reset servo error id.[%d]", MCMD_ERR_CLEAR_SERVO_ID);
    Serial.println(tmp_msg);
    return true;
  }

  // コマンド:MCMD_BOARD_TRANSMIT_ACTIVE (10005) UDP受信の通信周期制御をボード側主導に(デフォルト)
  if (a_meridim.sval[MRD_MASTER] == MCMD_BOARD_TRANSMIT_ACTIVE) {
    flg.udp_board_passive = false; // UDP送信をアクティブモードに
    flg.count_frame_reset = true;  // フレームの管理時計をリセットフラグをセット
    return true;
  }

  // コマンド:MCMD_EEPROM_ENTER_WRITE (10009) EEPROMの書き込みモードスタート
  if (a_meridim.sval[MRD_MASTER] == MCMD_EEPROM_ENTER_WRITE) {
    flg.eeprom_write_mode = true; // 書き込みモードのフラグをセット
    flg.count_frame_reset = true; // フレームの管理時計をリセットフラグをセット
    return true;
  }

  // コマンド:MCMD_EEPROM_SAVE_TRIM (10101) EEPROMに現在のサーボ値をTRIM値として書き込む
  if (a_meridim.sval[MRD_MASTER] == MCMD_EEPROM_SAVE_TRIM) {
    char tmp_msg[64];
    snprintf(tmp_msg, sizeof(tmp_msg), "cmd: save servo trim to EEPROM.[%d]", MCMD_EEPROM_SAVE_TRIM);
    Serial.println(tmp_msg);

    // 現在のEEPROM内容を読んでベースにする(ネットワーク設定等を保持)
    UnionEEPROM array_tmp = mrd_eeprom_read();

    // 受信したサーボ設定(config word + trim)をEEPROMマップの該当位置に上書き
    for (int i = 0; i < MRD_SV_SLOTS; i++) {
      array_tmp.sval[EEP_W_L_SV + i * 2]     = a_meridim.sval[MRD_L_ORIGIDX + i * 2];
      array_tmp.sval[EEP_W_L_SV + i * 2 + 1] = a_meridim.sval[MRD_L_ORIGIDX + 1 + i * 2];
      array_tmp.sval[EEP_W_R_SV + i * 2]     = a_meridim.sval[MRD_R_ORIGIDX + i * 2];
      array_tmp.sval[EEP_W_R_SV + i * 2 + 1] = a_meridim.sval[MRD_R_ORIGIDX + 1 + i * 2];
    }

    // CRCを再計算して格納
    array_tmp.usval[EEP_W_CRC] = mrd_crc16(array_tmp.usval, EEP_W_CRC);

    // 書き込み
    if (mrd_eeprom_write(array_tmp, EEPROM_PROTECT, a_serial)) {
      a_serial.println("write EEPROM succeed.");
    } else {
      a_serial.println("write EEPROM failed.");
      return false;
    }
    // ランタイムに即時反映(方向設定等を保存後すぐ有効化)
    mrd_eeprom_load_config(a_sv, a_serial);
    return true;
  }

  // コマンド:MCMD_EEPROM_LOAD_TRIM (10102) EEPROMからTRIM値を読み込んで設定
  if (a_meridim.sval[MRD_MASTER] == MCMD_EEPROM_LOAD_TRIM) {
    // EEPROMのデータを展開する
    mrd_eeprom_load_config(a_sv, Serial);

    // サーボをEEPROMのTRIM値で補正されたHOME(原点)に移動する
    for (int i = 0; i < MRD_SV_SLOTS; i++) {
      a_meridim.sval[MRD_L_ORIGIDX + 1 + i * 2] = 0; // L系統の目標値を原点に
      a_meridim.sval[MRD_R_ORIGIDX + 1 + i * 2] = 0; // R系統の目標値を原点に
      a_sv.ixl_tgt[i] = 0;                           //
      a_sv.ixr_tgt[i] = 0;
    }

    // サーボ動作を実行する
    if (!MODE_ESP32_STANDALONE) {
      mrd_servos_drive_lite(a_meridim, MOUNT_SV_TYPE_L, MOUNT_SV_TYPE_R, a_sv);
    }

    flg.count_frame_reset = true; // フレームの管理時計をリセットフラグをセット
    return true;
  }

  return false;
}

/// @brief Master Commandの第2群を実行する. 受信コマンドに基づき, 異なる処理を行う.
/// @param a_meridim 実行したいコマンドの入ったMeridim配列.(参照渡し)
/// @param a_flg_exe Meridimの受信成功判定フラグ.
/// @param a_sv サーボパラメータの構造体.(参照渡し)
/// @return コマンドを実行した場合はtrue, しなかった場合はfalseを返す.
bool execute_master_command_2(Meridim90Union &a_meridim, bool a_flg_exe, ServoParam &a_sv, HardwareSerial &a_serial) {
  if (!a_flg_exe) {
    return false;
  }
  // コマンド[90]: 1~999は MeridimのLength. デフォルトは90

  // コマンド:[0] 全サーボ脱力
  if (a_meridim.sval[MRD_MASTER] == 0) {
    mrd_servo_all_off(s_udp_meridim);
    return true;
  }

  // コマンド:[1] サーボオン 通常動作

  // コマンド:MCMD_SENSOR_YAW_CALIB(10002) IMU/AHRSのヨー軸リセット
  if (a_meridim.sval[MRD_MASTER] == MCMD_SENSOR_YAW_CALIB) {
    mrd_ahrs_set_yaw_origin(mrd_ahrs_get_yaw_source());
    char tmp_msg[64];
    snprintf(tmp_msg, sizeof(tmp_msg), "cmd: calibrate sensor's yaw.[%d]", MCMD_SENSOR_YAW_CALIB);
    Serial.println(tmp_msg);
    return true;
  }

  // コマンド:MCMD_BOARD_TRANSMIT_PASSIVE (10006) UDP受信の通信周期制御をPC側主導に(SSH的な動作)
  if (a_meridim.sval[MRD_MASTER] == MCMD_BOARD_TRANSMIT_PASSIVE) {
    flg.udp_board_passive = true; // UDP送信をパッシブモードに
    flg.count_frame_reset = true; // フレームの管理時計をリセットフラグをセット
    char tmp_msg[64];
    snprintf(tmp_msg, sizeof(tmp_msg), "cmd: enter passive mode.[%d]", MCMD_BOARD_TRANSMIT_PASSIVE);
    Serial.println(tmp_msg);
    return true;
  }

  // コマンド:MCMD_FRAMETIMER_RESET) (10007) フレームカウンタを現在時刻にリセット
  if (a_meridim.sval[MRD_MASTER] == MCMD_FRAMETIMER_RESET) {
    flg.count_frame_reset = true; // フレームの管理時計をリセットフラグをセット
    return true;
  }

  // コマンド:MCMD_BOARD_STOP_DURING (10008) ボードの末端処理を指定時間だけ止める.
  if (a_meridim.sval[MRD_MASTER] == MCMD_BOARD_STOP_DURING) {
    flg.stop_board_during = true; // ボードの処理停止フラグをセット
    // ボードの末端処理をmeridim[2]ミリ秒だけ止める.

    char tmp_msg[80];
    snprintf(tmp_msg, sizeof(tmp_msg), "cmd: stop ESP32's processing during %d ms.[%d]",
             (int)a_meridim.sval[MRD_STOP_FRAMES], MCMD_BOARD_STOP_DURING);
    Serial.println(tmp_msg);

    for (int i = 0; i < int(a_meridim.sval[MRD_STOP_FRAMES]); i++) {
      delay(1);
    }
    flg.stop_board_during = false; // ボードの処理停止フラグをクリア
    flg.count_frame_reset = true;  // フレームの管理時計をリセットフラグをセット
    return true;
  }
  return false;
}

/// @brief Master Commandの第3群を実行する. 受信コマンドに基づき, 異なる処理を行う.
/// @param a_meridim 実行したいコマンドの入ったMeridim配列.(参照渡し)
/// @param a_flg_exe Meridimの受信成功判定フラグ.
/// @param a_sv サーボパラメータの構造体.(参照渡し)
/// @return コマンドを実行した場合はtrue, しなかった場合はfalseを返す.
bool execute_master_command_3(Meridim90Union &a_meridim, bool a_flg_exe, ServoParam &a_sv, HardwareSerial &a_serial) {
  if (!a_flg_exe) {
    return false;
  }
  // コマンド[90]: 1~999は MeridimのLength. デフォルトは90

  // コマンド:[0] 全サーボ脱力
  if (a_meridim.sval[MRD_MASTER] == 0) {
    mrd_servo_all_off(a_meridim);
    return true;
  }

  // コマンド:[1] サーボオン 通常動作

  // コマンド:MCMD_START_TRIM_SETTIN (10003) TRIM設定のスタート(Meridian_console連携)
  if (a_meridim.sval[MRD_MASTER] == MCMD_START_TRIM_SETTING) {

    // EEPROMのデータを展開する
    mrd_eeprom_load_config(a_sv, Serial);

    // サーボをEEPROMのTRIM値で補正されたHOME(原点)に移動する
    for (int i = 0; i < MRD_SV_SLOTS; i++) {
      a_meridim.sval[MRD_L_ORIGIDX + 1 + i * 2] = 0; // L系統の目標値を原点に
      a_meridim.sval[MRD_R_ORIGIDX + 1 + i * 2] = 0; // R系統の目標値を原点に
      a_sv.ixl_tgt_past[i] = a_sv.ixl_tgt[i];        // 前回のdegreeをキープ
      a_sv.ixr_tgt_past[i] = a_sv.ixr_tgt[i];
      a_sv.ixl_tgt[i] = 0; //
      a_sv.ixr_tgt[i] = 0;
    }

    // サーボ動作を実行する
    if (!MODE_ESP32_STANDALONE) {
      mrd_servos_drive_lite(a_meridim, MOUNT_SV_TYPE_L, MOUNT_SV_TYPE_R, a_sv);
    }

    // サーボの目標値として現在のTRIM値をセットする
    for (int i = 0; i < MRD_SV_SLOTS; i++) {
      a_meridim.sval[MRD_L_ORIGIDX + 1 + i * 2] = a_sv.ixl_trim[i];
      a_meridim.sval[MRD_R_ORIGIDX + 1 + i * 2] = a_sv.ixr_trim[i];
    }

    // サーボのTRIM値をゼロリセットする
    for (int i = 0; i < MRD_SV_SLOTS; i++) {
      a_sv.ixl_trim[i] = 0;
      a_sv.ixr_trim[i] = 0;
    }

    // サーボ動作を実行する. サーボはTRIM値を0としつつ, tgtとしてこれまでのTRIM値の角度をキープする
    if (!MODE_ESP32_STANDALONE) {
      mrd_servos_drive_lite(a_meridim, MOUNT_SV_TYPE_L, MOUNT_SV_TYPE_R, a_sv); // サーボ動作を実行する
    }

    // サーボ設定を格納する
    for (int i = 0; i < MRD_SV_SLOTS; i++) {
      a_meridim.sval[MRD_L_ORIGIDX + i * 2] = a_sv.ixl_trim[i];
      a_meridim.sval[MRD_R_ORIGIDX + i * 2] = a_sv.ixr_trim[i];
    }

    // サーボの設定値とTRIM値をPCに送信する
    UnionEEPROM array_tmp = mrd_eeprom_read();
    for (int i = 0; i < MRDM_LEN; i++) {
      a_meridim.sval[i] = array_tmp.saval[1][i];
    }
    a_meridim.sval[MRD_MASTER] = MCMD_EEPROM_BOARDTOPC_DATA1;

    a_serial.println("send:");
    for (int i = 0; i < MRDM_LEN; i++) {
      a_serial.print(a_meridim.sval[i]);
      a_serial.print(",");
    }
    a_serial.println();

    char tmp_msg[80];
    snprintf(tmp_msg, sizeof(tmp_msg), "cmd: enter trim setting mode and send EEPROM[1][*] to PC.[%d]", MCMD_START_TRIM_SETTING);
    Serial.println(tmp_msg);
    return true;
  }

  // コマンド:MCMD_EEPROM_BOARDTOPC_DATA0/1/2 (10200-10202) EEPROMの[n][*]をボードからPCにMeridimで送信
  int16_t cmd = a_meridim.sval[MRD_MASTER];
  if (cmd >= MCMD_EEPROM_BOARDTOPC_DATA0 && cmd <= MCMD_EEPROM_BOARDTOPC_DATA2) {
    int idx = cmd - MCMD_EEPROM_BOARDTOPC_DATA0; // 0, 1, or 2
    UnionEEPROM array_tmp = mrd_eeprom_read();
    for (int i = 0; i < MRDM_LEN; i++) {
      a_meridim.sval[i] = array_tmp.saval[idx][i];
    }
    a_meridim.sval[MRD_MASTER] = cmd;

    char tmp_msg[64];
    snprintf(tmp_msg, sizeof(tmp_msg), "cmd: send EEPROM[%d][*] to PC.[%d]", idx, cmd);
    Serial.println(tmp_msg);
    return true;
  }

  // コマンド:MCMD_EEPROM_PCTOBOARD_DATA0/1/2 (10300-10302) PCからボードへ3パケットでEEPROM全データを送信
  {
    static UnionEEPROM s_pctoboard_buf  = {0};
    static uint8_t     s_pctoboard_rcvd = 0; // bitmask: bit0=pkt0, bit1=pkt1, bit2=pkt2

    int16_t pcmd = a_meridim.sval[MRD_MASTER];
    if (pcmd >= MCMD_EEPROM_PCTOBOARD_DATA0 && pcmd <= MCMD_EEPROM_PCTOBOARD_DATA2) {
      int idx         = pcmd - MCMD_EEPROM_PCTOBOARD_DATA0;          // 0, 1, or 2
      int eeprom_base = idx * (MRDM_LEN - 2) + 1;                   // 1, 89, or 177
      for (int i = 1; i < MRDM_LEN - 1; i++) {
        s_pctoboard_buf.sval[eeprom_base + i - 1] = a_meridim.sval[i];
      }
      s_pctoboard_rcvd |= (1 << idx);

      char tmp_msg[64];
      snprintf(tmp_msg, sizeof(tmp_msg), "cmd: rcvd EEPROM block %d from PC.[%d]", idx, pcmd);
      Serial.println(tmp_msg);

      if (s_pctoboard_rcvd == 0x07) { // all 3 packets received
        s_pctoboard_rcvd = 0;
        s_pctoboard_buf.usval[EEP_W_HEADER] = ((uint16_t)EEP_FIXED_WORDS << 8) | EEP_INIT_ID;
        s_pctoboard_buf.usval[EEP_W_CRC]    = mrd_crc16(s_pctoboard_buf.usval, EEP_W_CRC);
        if (mrd_eeprom_write(s_pctoboard_buf, EEPROM_PROTECT, a_serial)) {
          a_serial.println("write full EEPROM from PC: succeed.");
        } else {
          a_serial.println("write full EEPROM from PC: failed.");
          return false;
        }
      }
      return true;
    }
  }

  return false;
}
