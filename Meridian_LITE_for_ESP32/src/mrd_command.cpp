// mrd_command.cpp
// コマンド処理関数の実装

// ヘッダファイルの読み込み
#include "mrd_command.h"

#include "mrd_eeprom.h"
#include "mrd_servo.h"
#include "mrd_wire0.h"

// ライブラリ導入

//==================================================================================================
// 共有リソース
//==================================================================================================
extern SemaphoreHandle_t ahrs_mutex; // AHRSデータアクセス用mutex

//==================================================================================================
//  コマンド処理
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
                              MrdFlags &a_flg, MERIDIANFLOW::Meridian &a_mrd) {
  if (!a_flg_exe) {
    return false;
  }
  // Command[90]: 1~999 は Meridim Length. デフォルトは 90

  // コマンド:MCMD_ERR_CLEAR_SERVO_ID (10004) 通信エラーサーボIDをクリア
  if (a_meridim.sval[MRD_MASTER] == MCMD_ERR_CLEAR_SERVO_ID) {
    a_meridim.bval[MRD_ERR_l] = 0;
    for (int i = 0; i < IXL_MAX; i++) {
      a_sv.ixl_err[i] = 0;
    }
    for (int i = 0; i < IXR_MAX; i++) {
      a_sv.ixr_err[i] = 0;
    }
    String msg_tmp = "cmd: reset servo error id.[" + String(MCMD_ERR_CLEAR_SERVO_ID) + "]";
    a_serial.println(msg_tmp);
    return true;
  }

  // コマンド:MCMD_BOARD_TRANSMIT_ACTIVE (10005) UDP受信タイミング制御をボード主導に設定 (デフォルト)
  if (a_meridim.sval[MRD_MASTER] == MCMD_BOARD_TRANSMIT_ACTIVE) {
    a_flg.udp_board_passive = false; // UDP送信をアクティブモードに設定
    a_flg.count_frame_reset = true;  // フレーム管理タイマーリセットフラグをセット
    return true;
  }

  // コマンド:MCMD_EEPROM_ENTER_WRITE (10009) EEPROM書き込みモードを開始
  if (a_meridim.sval[MRD_MASTER] == MCMD_EEPROM_ENTER_WRITE) {
    a_flg.eeprom_write_mode = true; // 書き込みモードフラグをセット
    a_flg.count_frame_reset = true; // フレーム管理タイマーリセットフラグをセット
    return true;
  }

  // コマンド:MCMD_EEPROM_SAVE_TRIM (10101) 現在のサーボ値をTRIM値としてEEPROMに書き込む
  if (a_meridim.sval[MRD_MASTER] == MCMD_EEPROM_SAVE_TRIM) {
    String msg_tmp = "cmd: set EEPROM data from current trim.[" + String(MCMD_EEPROM_SAVE_TRIM) + "]";
    a_serial.println(msg_tmp);

    // 空のUnionEEPROM構造体を作成して初期化
    UnionEEPROM array_tmp = {0};

    // サーボ設定をsaval[1]にコピー
    for (int i = 0; i < a_sv.num_max; i++) {
      array_tmp.saval[1][20 + i * 2] = a_meridim.sval[20 + i * 2];
      array_tmp.saval[1][21 + i * 2] = a_meridim.sval[21 + i * 2];
      array_tmp.saval[1][50 + i * 2] = a_meridim.sval[50 + i * 2];
      array_tmp.saval[1][51 + i * 2] = a_meridim.sval[51 + i * 2];
    }

    // 書き込みデータを作成して書き込み
    if (mrd_eeprom_write(array_tmp, EEPROM_PROTECT, a_serial, a_flg)) {
      a_serial.println("write EEPROM succeed.");
    } else {
      a_serial.println("write EEPROM failed.");
      return false;
    }
    return true;
  }

  // コマンド:MCMD_EEPROM_LOAD_TRIM (10102) EEPROMからTRIM値を読み込んで設定を適用
  if (a_meridim.sval[MRD_MASTER] == MCMD_EEPROM_LOAD_TRIM) {
    // EEPROMデータを展開
    mrd_eeprom_load_servosettings(a_sv, true, a_serial);

    // EEPROM TRIM値で補正されたHOME (原点) にサーボを移動
    for (int i = 0; i < MRD_SERVO_SLOTS; i++) {
      a_meridim.sval[MRD_L_ORIGIDX + 1 + i * 2] = 0; // L系統ターゲットを原点に設定
      a_meridim.sval[MRD_R_ORIGIDX + 1 + i * 2] = 0; // R系統ターゲットを原点に設定
      a_sv.ixl_tgt[i] = 0;                           //
      a_sv.ixr_tgt[i] = 0;
    }

    // サーボ移動を実行
    if (!MODE_ESP32_STANDALONE) {
      mrd_servo_drive_lite(a_meridim, (ServoType)MOUNT_SERVO_TYPE_L, (ServoType)MOUNT_SERVO_TYPE_R,
                           a_sv, a_ics_L, a_ics_R, a_mrd);
    }

    a_flg.count_frame_reset = true; // フレーム管理タイマーリセットフラグをセット
    return true;
  }

  return false;
}

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
                              HardwareSerial &a_serial, MrdFlags &a_flg) {
  if (!a_flg_exe) {
    return false;
  }
  // Command[90]: 1~999 は Meridim Length. デフォルトは 90

  // コマンド:[0] 全サーボ電源オフ
  if (a_meridim.sval[MRD_MASTER] == 0) {
    mrd_servo_all_off(a_meridim);
    return true;
  }

  // コマンド:[1] サーボオン, 通常動作

  // コマンド:MCMD_SENSOR_YAW_CALIB(10002) IMU/AHRSのヨー軸リセット
  if (a_meridim.sval[MRD_MASTER] == MCMD_SENSOR_YAW_CALIB) {
    // mutex保護下でAHRSデータにアクセス
    if (xSemaphoreTake(ahrs_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      mrd_wire0_calibrate_yaw_origin();
      xSemaphoreGive(ahrs_mutex);
    }
    String msg_tmp = "cmd: calibrate sensor's yaw.[" + String(MCMD_SENSOR_YAW_CALIB) + "]";
    a_serial.println(msg_tmp);
    return true;
  }

  // コマンド:MCMD_BOARD_TRANSMIT_PASSIVE (10006) UDP受信タイミング制御をPC主導に設定 (SSH的動作)
  if (a_meridim.sval[MRD_MASTER] == MCMD_BOARD_TRANSMIT_PASSIVE) {
    a_flg.udp_board_passive = true; // UDP送信をパッシブモードに設定
    a_flg.count_frame_reset = true; // フレーム管理タイマーリセットフラグをセット
    String msg_tmp = "cmd: enter passive mode.[" + String(MCMD_BOARD_TRANSMIT_PASSIVE) + "]";
    a_serial.println(msg_tmp);
    return true;
  }

  // コマンド:MCMD_FRAMETIMER_RESET (10007) フレームカウンターを現在時刻にリセット
  if (a_meridim.sval[MRD_MASTER] == MCMD_FRAMETIMER_RESET) {
    a_flg.count_frame_reset = true; // フレーム管理タイマーリセットフラグをセット
    return true;
  }

  // コマンド:MCMD_BOARD_STOP_DURING (10008) 指定時間ボード終端処理を停止
  if (a_meridim.sval[MRD_MASTER] == MCMD_BOARD_STOP_DURING) {
    a_flg.stop_board_during = true; // ボード処理停止フラグをセット
    // meridim[2]ミリ秒間ボード終端処理を停止

    String msg_tmp = "cmd: stop ESP32's processing during " + String(int(a_meridim.sval[MRD_STOP_FRAMES])) + " ms.[" + String(MCMD_BOARD_STOP_DURING) + "]";
    a_serial.println(msg_tmp);

    for (int i = 0; i < int(a_meridim.sval[MRD_STOP_FRAMES]); i++) {
      delay(1);
    }
    a_flg.stop_board_during = false; // ボード処理停止フラグをクリア
    a_flg.count_frame_reset = true;  // フレーム管理タイマーリセットフラグをセット
    return true;
  }
  return false;
}

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
                              MrdFlags &a_flg, MERIDIANFLOW::Meridian &a_mrd) {
  if (!a_flg_exe) {
    return false;
  }
  // Command[90]: 1~999 は Meridim Length. デフォルトは 90

  // コマンド:[0] 全サーボ電源オフ
  if (a_meridim.sval[MRD_MASTER] == 0) {
    mrd_servo_all_off(a_meridim);
    return true;
  }

  // コマンド:[1] サーボオン, 通常動作

  // コマンド:MCMD_START_TRIM_SETTIN (10100) TRIM設定を開始 (Meridian_console連携)
  if (a_meridim.sval[MRD_MASTER] == MCMD_START_TRIM_SETTING) {

    // EEPROMデータを展開
    mrd_eeprom_load_servosettings(a_sv, true, a_serial);

    // EEPROM TRIM値で補正されたHOME (原点) にサーボを移動
    for (int i = 0; i < MRD_SERVO_SLOTS; i++) {
      a_meridim.sval[MRD_L_ORIGIDX + 1 + i * 2] = 0; // L系統ターゲットを原点に設定
      a_meridim.sval[MRD_R_ORIGIDX + 1 + i * 2] = 0; // R系統ターゲットを原点に設定
      a_sv.ixl_tgt_past[i] = a_sv.ixl_tgt[i];        // 前回の角度を保持
      a_sv.ixr_tgt_past[i] = a_sv.ixr_tgt[i];
      a_sv.ixl_tgt[i] = 0; //
      a_sv.ixr_tgt[i] = 0;
    }

    // サーボ移動を実行
    if (!MODE_ESP32_STANDALONE) {
      mrd_servo_drive_lite(a_meridim, (ServoType)MOUNT_SERVO_TYPE_L, (ServoType)MOUNT_SERVO_TYPE_R,
                           a_sv, a_ics_L, a_ics_R, a_mrd);
    }

    // 現在のTRIM値をサーボターゲット値として設定
    for (int i = 0; i < MRD_SERVO_SLOTS; i++) {
      a_meridim.sval[MRD_L_ORIGIDX + 1 + i * 2] = a_sv.ixl_trim[i];
      a_meridim.sval[MRD_R_ORIGIDX + 1 + i * 2] = a_sv.ixr_trim[i];
    }

    // サーボTRIM値をゼロリセット
    for (int i = 0; i < MRD_SERVO_SLOTS; i++) {
      a_sv.ixl_trim[i] = 0;
      a_sv.ixr_trim[i] = 0;
    }

    // サーボ移動を実行. TRIM値が0の状態で, 前回TRIM値の角度をtgtとしてサーボは保持
    if (!MODE_ESP32_STANDALONE) {
      mrd_servo_drive_lite(a_meridim, (ServoType)MOUNT_SERVO_TYPE_L, (ServoType)MOUNT_SERVO_TYPE_R,
                           a_sv, a_ics_L, a_ics_R, a_mrd); // サーボ動作を実行する
    }

    // サーボ設定を格納 ####(誤りの可能性あり)
    for (int i = 0; i < MRD_SERVO_SLOTS; i++) {
      a_meridim.sval[MRD_L_ORIGIDX + i * 2] = a_sv.ixl_trim[i];
      a_meridim.sval[MRD_R_ORIGIDX + i * 2] = a_sv.ixr_trim[i];
    }

    // サーボ設定とTRIM値をPCに送信
    UnionEEPROM array_tmp = mrd_eeprom_read();
    for (int i = 0; i < MRDM_LEN; i++) {
      a_meridim.sval[i] = array_tmp.saval[1][i];
    }
    a_meridim.sval[MRD_MASTER] = MCMD_EEPROM_BOARDTOPC_DATA1;

    // Serial出力を一括で行う (90回のprint呼び出しを削減)
    a_serial.print("send: ");
    String buf;
    buf.reserve(MRDM_LEN * 7); // 各値は最大6桁+カンマ
    for (int i = 0; i < MRDM_LEN; i++) {
      buf += String(a_meridim.sval[i]);
      buf += ',';
    }
    a_serial.println(buf);

    String msg_tmp = "cmd: enter trim setting mode and send EEPROM[1][*] to PC.[" + String(MCMD_START_TRIM_SETTING) + "]";
    a_serial.println(msg_tmp);
    return true;
  }

  // コマンド:MCMD_EEPROM_BOARDTOPC_DATA0(10200) ボードからPCへEEPROM[0][*]をMeridim経由で送信
  if (a_meridim.sval[MRD_MASTER] == MCMD_EEPROM_BOARDTOPC_DATA0) {
    // eepromをs_meridimに代入
    UnionEEPROM array_tmp = mrd_eeprom_read();
    for (int i = 0; i < MRDM_LEN; i++) {
      a_meridim.sval[i] = array_tmp.saval[0][i];
    }
    a_meridim.sval[MRD_MASTER] = MCMD_EEPROM_BOARDTOPC_DATA0;

    String msg_tmp = "cmd: enter trim setting mode and send EEPROM[0][*] to PC.[" + String(MCMD_EEPROM_BOARDTOPC_DATA0) + "]";
    a_serial.println(msg_tmp);
    return true;
  }

  // コマンド:MCMD_EEPROM_BOARDTOPC_DATA1(10201) ボードからPCへEEPROM[1][*]をMeridim経由で送信
  if (a_meridim.sval[MRD_MASTER] == MCMD_EEPROM_BOARDTOPC_DATA1) {
    // eepromをs_meridimに代入
    UnionEEPROM array_tmp = mrd_eeprom_read();
    for (int i = 0; i < MRDM_LEN; i++) {
      a_meridim.sval[i] = array_tmp.saval[1][i];
    }
    a_meridim.sval[MRD_MASTER] = MCMD_EEPROM_BOARDTOPC_DATA1;

    String msg_tmp = "cmd: enter trim setting mode and send EEPROM[1][*] to PC.[" + String(MCMD_EEPROM_BOARDTOPC_DATA1) + "]";
    a_serial.println(msg_tmp);
    return true;
  }

  // コマンド:MCMD_EEPROM_BOARDTOPC_DATA2(10202) ボードからPCへEEPROM[2][*]をMeridim経由で送信
  if (a_meridim.sval[MRD_MASTER] == MCMD_EEPROM_BOARDTOPC_DATA2) {
    // eepromをs_meridimに代入
    UnionEEPROM array_tmp = mrd_eeprom_read();
    for (int i = 0; i < MRDM_LEN; i++) {
      a_meridim.sval[i] = array_tmp.saval[2][i];
    }
    a_meridim.sval[MRD_MASTER] = MCMD_EEPROM_BOARDTOPC_DATA2;

    String msg_tmp = "cmd: enter trim setting mode and send EEPROM[2][*] to PC.[" + String(MCMD_EEPROM_BOARDTOPC_DATA2) + "]";
    a_serial.println(msg_tmp);
    return true;
  }

  return false;
}
