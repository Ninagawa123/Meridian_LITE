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

/// @brief Master Commandの第1群を実行する. 受信コマンドに基づき, 異なる処理を行う.
/// @param a_meridim 実行したいコマンドの入ったMeridim配列を渡す.
/// @param a_flg_exe Meridimの受信成功判定フラグを渡す.
/// @return コマンドを実行した場合はtrue, しなかった場合はfalseを返す.
bool execute_master_command_1(Meridim90Union a_meridim, bool a_flg_exe, HardwareSerial &a_serial) {
  if (!a_flg_exe) {
    return false;
  }
  // コマンド[90]: 1~999は MeridimのLength. デフォルトは90

  // コマンド:MCMD_ERR_CLEAR_SERVO_ID (10004) 通信エラーサーボIDのクリア
  if (a_meridim.sval[MRD_MASTER] == MCMD_ERR_CLEAR_SERVO_ID) {
    r_udp_meridim.bval[MRD_ERR_l] = 0;
    s_udp_meridim.bval[MRD_ERR_l] = 0;
    for (int i = 0; i < IXL_MAX; i++) {
      sv.ixl_err[i] = 0;
    }
    for (int i = 0; i < IXR_MAX; i++) {
      sv.ixr_err[i] = 0;
    }
    a_serial.println("Servo Error ID reset.");
    return true;
  }

  // コマンド:MCMD_BOARD_TRANSMIT_ACTIVE (10005) UDP受信の通信周期制御をボード側主導に（デフォルト）
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
    a_serial.println("Set EEPROM data from current trim.");

    // 空のUnionEEPROM構造体を作成し初期化
    UnionEEPROM array_tmp = {0};

    Serial.println("a_meridim:");
    for (int i = 0; i < 90; i++) {
      Serial.print(a_meridim.sval[i]);
      Serial.print(", ");
    }
    Serial.println();

    // サーボの設定情報をsaval[1]にコピー
    for (int i = 0; i < sv.num_max; i++) {
      array_tmp.saval[1][20 + i * 2] = a_meridim.sval[20 + i * 2];
      array_tmp.saval[1][21 + i * 2] = a_meridim.sval[21 + i * 2];
      array_tmp.saval[1][50 + i * 2] = a_meridim.sval[50 + i * 2]; // - int(sv.ixl_trim[i] * 100);
      array_tmp.saval[1][51 + i * 2] = a_meridim.sval[51 + i * 2]; // - int(sv.ixr_trim[i] * 100);
    }

    Serial.println("array_tmp:");
    for (int i = 0; i < 90; i++) {
      Serial.print(array_tmp.saval[1][i]);
      Serial.print(", ");
    }
    Serial.println();

    // UnionEEPROM array_tmp = {0};

    // for (int i = 0; i < sv.num_max; ++i) {
    //   //------------------------------------------------------------------
    //   // 1) 左右サーボ共通 ― ビットフィールドを組み立て
    //   //------------------------------------------------------------------
    //   uint16_t l_tmp = 0;
    //   uint16_t r_tmp = 0;

    //   // bit0 : マウント有無
    //   if (sv.ixl_mount[i])
    //     l_tmp |= 0x0001;
    //   if (sv.ixr_mount[i])
    //     r_tmp |= 0x0001;

    //   // bits1-7 : サーボ ID (7 bit)
    //   l_tmp |= static_cast<uint16_t>(sv.ixl_id[i] & 0x7F) << 1;
    //   r_tmp |= static_cast<uint16_t>(sv.ixr_id[i] & 0x7F) << 1;

    //   // bit8 : 回転方向 (1 = CW, 0 = CCW) ※CCW を −1 で受けている想定
    //   if (sv.ixl_cw[i] > 0)
    //     l_tmp |= 0x0100;
    //   if (sv.ixr_cw[i] > 0)
    //     r_tmp |= 0x0100;

    //   //------------------------------------------------------------------
    //   // 2) 16-bit ワードへ格納
    //   //------------------------------------------------------------------
    //   array_tmp.saval[1][20 + i * 2] = l_tmp;
    //   array_tmp.saval[1][50 + i * 2] = r_tmp;

    //   //------------------------------------------------------------------
    //   // 3) トリム角度 (degree ×100) を隣接ワードへ格納
    //   //     例: 12.34° → 1234
    //   //------------------------------------------------------------------
    //   array_tmp.saval[1][21 + i * 2] = static_cast<uint16_t>(std::lround(sv.ixl_trim[i] * 100.0f));
    //   array_tmp.saval[1][51 + i * 2] = static_cast<uint16_t>(std::lround(sv.ixr_trim[i] * 100.0f));
    // }

    // bool mrd_eeprom_load_servosettings(ServoParam & a_sv, bool a_monitor, HardwareSerial &a_serial) {
    //   a_serial.println("Load and set servo settings from EEPROM.");
    //   UnionEEPROM array_tmp = mrd_eeprom_read();
    //   for (int i = 0; i < a_sv.num_max; i++) {
    //     // 各サーボのマウント有無
    //     a_sv.ixl_mount[i] = static_cast<bool>(array_tmp.saval[1][20 + i * 2] & 0x0001); // bit0:マウント有無
    //     a_sv.ixr_mount[i] = static_cast<bool>(array_tmp.saval[1][50 + i * 2] & 0x0001); // bit0:マウント有無
    //     // 各サーボの実サーボ呼び出しID番号
    //     a_sv.ixl_id[i] = static_cast<uint8_t>(array_tmp.saval[1][20 + i * 2] >> 1 & 0x007F); // bit1–7:サーボID
    //     a_sv.ixr_id[i] = static_cast<uint8_t>(array_tmp.saval[1][50 + i * 2] >> 1 & 0x007F); // bit1–7:サーボID
    //     // 各サーボの回転方向（正転・逆転）
    //     a_sv.ixl_cw[i] = static_cast<int8_t>((array_tmp.saval[1][20 + i * 2] >> 8) & 0x0001) ? 1 : -1; // bit8:回転方向
    //     a_sv.ixr_cw[i] = static_cast<int8_t>((array_tmp.saval[1][50 + i * 2] >> 8) & 0x0001) ? 1 : -1; // bit8:回転方向
    //     // 各サーボの直立デフォルト角度,トリム値(degree小数2桁までを100倍した値で格納されているものを展開)
    //     a_sv.ixl_trim[i] = array_tmp.saval[1][21 + i * 2] / 100.0f;
    //     a_sv.ixr_trim[i] = array_tmp.saval[1][51 + i * 2] / 100.0f;

    //   if (a_monitor) {
    //     a_serial.print("L-idx:");
    //     a_serial.print(mrd_pddstr(i, 2, 0, false));
    //     a_serial.print(", id:");
    //     a_serial.print(mrd_pddstr(sv.ixl_id[i], 2, 0, false));
    //     a_serial.print(", mt:");
    //     a_serial.print(mrd_pddstr(sv.ixl_mount[i], 1, 0, false));
    //     a_serial.print(", cw:");
    //     a_serial.print(mrd_pddstr(sv.ixl_cw[i], 1, 0, true));
    //     a_serial.print(", trm:");
    //     a_serial.print(mrd_pddstr(sv.ixl_trim[i], 7, 2, true));
    //     a_serial.print("  R-idx: ");
    //     a_serial.print(mrd_pddstr(i, 2, 0, false));
    //     a_serial.print(", id:");
    //     a_serial.print(mrd_pddstr(sv.ixr_id[i], 2, 0, false));
    //     a_serial.print(", mt:");
    //     a_serial.print(mrd_pddstr(sv.ixr_mount[i], 1, 0, false));
    //     a_serial.print(", cw:");
    //     a_serial.print(mrd_pddstr(sv.ixr_cw[i], 1, 0, true));
    //     a_serial.print(", trm:");
    //     a_serial.println(mrd_pddstr(sv.ixr_trim[i], 7, 2, true));
    //   }
    // }
    // return true;

    // デバッグ用の追加表示
    // a_serial.println("EEPROM data to save (first few values):");
    // for (int i = 0; i < 3; i++) {
    //   a_serial.print("L");
    //   a_serial.print(i);
    //   a_serial.print(" Settings: ");
    //   a_serial.print(array_tmp.saval[1][20 + i * 2]);
    //   a_serial.print(", Trim: ");
    //   a_serial.println(array_tmp.saval[1][21 + i * 2]);
    // }

    // 書き込みデータの作成と書き込み
    if (mrd_eeprom_write(array_tmp, EEPROM_PROTECT, Serial)) {
      a_serial.println("Write EEPROM succeed.");
      // a_serial.print("EEPROM data updated at address 1: ");

      // // // 一部の値を確認として表示
      // UnionEEPROM check_data = mrd_eeprom_read();
      // for (int i = 0; i < 3; i++) {
      //   a_serial.print("[");
      //   a_serial.print(20 + i * 2);
      //   a_serial.print("]=");
      //   a_serial.print(check_data.saval[1][20 + i * 2]);
      //   a_serial.print(" ");
      // }
      // a_serial.println();
    } else {
      a_serial.println("Write EEPROM failed.");
      return false;
    }
    return true;
  }

  // コマンド:MCMD_EEPROM_LOAD_TRIM (10102) EEPROMからTRIM値を読み込んで設定
  if (a_meridim.sval[MRD_MASTER] == MCMD_EEPROM_LOAD_TRIM) {
    mrd_eeprom_load_servosettings(sv, true, Serial);
    flg.count_frame_reset = true; // フレームの管理時計をリセットフラグをセット
    return true;
  }

  return false;
}

/// @brief Master Commandの第2群を実行する. 受信コマンドに基づき, 異なる処理を行う.
/// @param a_meridim 実行したいコマンドの入ったMeridim配列を渡す.
/// @param a_flg_exe Meridimの受信成功判定フラグを渡す.
/// @return コマンドを実行した場合はtrue, しなかった場合はfalseを返す.
bool execute_master_command_2(Meridim90Union a_meridim, bool a_flg_exe, HardwareSerial &a_serial) {
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
    ahrs.yaw_origin = ahrs.yaw_source;
    a_serial.println("cmd: yaw reset.");
    return true;
  }

  // コマンド:MCMD_BOARD_TRANSMIT_PASSIVE (10006) UDP受信の通信周期制御をPC側主導に（SSH的な動作）
  if (a_meridim.sval[MRD_MASTER] == MCMD_BOARD_TRANSMIT_PASSIVE) {
    flg.udp_board_passive = true; // UDP送信をパッシブモードに
    flg.count_frame_reset = true; // フレームの管理時計をリセットフラグをセット
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
    a_serial.print("Stop ESP32's processing during ");
    a_serial.print(int(a_meridim.sval[MRD_STOP_FRAMES]));
    a_serial.println(" ms.");
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
/// @param a_meridim 実行したいコマンドの入ったMeridim配列を渡す.
/// @param a_flg_exe Meridimの受信成功判定フラグを渡す.
/// @return コマンドを実行した場合はtrue, しなかった場合はfalseを返す.
bool execute_master_command_3(Meridim90Union a_r_meridim, Meridim90Union &a_s_meridim, bool a_flg_exe, HardwareSerial &a_serial) {
  if (!a_flg_exe) {
    return false;
  }
  // コマンド[90]: 1~999は MeridimのLength. デフォルトは90

  // コマンド:[0] 全サーボ脱力
  if (a_r_meridim.sval[MRD_MASTER] == 0) {
    mrd_servo_all_off(s_udp_meridim);
    return true;
  }

  // コマンド:[1] サーボオン 通常動作

  // コマンド:MCMD_EEPROM_BOARDTOPC_DATA2(10200) EEPROMの[0][*]をボードからPCにMeridimで送信する
  if (a_r_meridim.sval[MRD_MASTER] == MCMD_EEPROM_BOARDTOPC_DATA0) {
    // eepromをs_meridimに代入する
    UnionEEPROM array_tmp = mrd_eeprom_read();
    for (int i = 0; i < MRDM_LEN; i++) {
      a_s_meridim.sval[i] = array_tmp.saval[0][i];
    }
    a_s_meridim.sval[MRD_MASTER] = MCMD_EEPROM_BOARDTOPC_DATA0;
    a_serial.println("Read EEPROM[0][*] and send to PC.");
    return true;
  }

  // コマンド:MCMD_EEPROM_BOARDTOPC_DATA2(10201) EEPROMの[1][*]をボードからPCにMeridimで送信する
  if (a_r_meridim.sval[MRD_MASTER] == MCMD_EEPROM_BOARDTOPC_DATA1) {
    // eepromをs_meridimに代入する
    UnionEEPROM array_tmp = mrd_eeprom_read();
    for (int i = 0; i < MRDM_LEN; i++) {
      a_s_meridim.sval[i] = array_tmp.saval[1][i];
    }
    a_s_meridim.sval[MRD_MASTER] = MCMD_EEPROM_BOARDTOPC_DATA1;

    Serial.println("revd:");
    for (int i = 0; i < MRDM_LEN; i++) {
      Serial.print(a_r_meridim.sval[i]);
      Serial.print(",");
    }
    Serial.println();

    Serial.println("send:");
    for (int i = 0; i < MRDM_LEN; i++) {
      Serial.print(a_s_meridim.sval[i]);
      Serial.print(",");
    }
    Serial.println();

    a_serial.println("Read EEPROM[1][*] and send to PC.");
    return true;
  }

  // コマンド:MCMD_EEPROM_BOARDTOPC_DATA2(10202) EEPROMの[2][*]をボードからPCにMeridimで送信する
  if (a_r_meridim.sval[MRD_MASTER] == MCMD_EEPROM_BOARDTOPC_DATA2) {
    // eepromをs_meridimに代入する
    UnionEEPROM array_tmp = mrd_eeprom_read();
    for (int i = 0; i < MRDM_LEN; i++) {
      a_s_meridim.sval[i] = array_tmp.saval[2][i];
    }
    a_s_meridim.sval[MRD_MASTER] = MCMD_EEPROM_BOARDTOPC_DATA2;
    a_serial.println("Read EEPROM[2][*] and send to PC.");
    return true;
  }

  return false;
}

#endif // __MERIDIAN_COMMAND_H__