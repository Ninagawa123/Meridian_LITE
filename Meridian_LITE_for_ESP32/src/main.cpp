#ifndef __MERIDIAN_LITE_MAIN__
#define __MERIDIAN_LITE_MAIN__

#define VERSION "Meridian_LITE_v1.2.0_2026_10.09" // バージョン表示

/// @file    Meridian_LITE_for_ESP32/src/main.cpp
/// @brief   Meridian is a system that smartly realizes the digital twin of a robot.
/// @details Meridian_LITE for Meridian Board -LITE- with ESP32DecKitC.
///
/// This code is licensed under the MIT License.
/// Copyright (c) 2022 Izumi Ninagawa & Project Meridian

//==================================================================================================
//  初期設定
//==================================================================================================

// ヘッダファイルの読み込み
#include <Arduino.h>

#include "config.h"
#include "keys.h"
#include "main.h"

#include "mrd_bt_pad.h"
#include "mrd_command.h"
#include "mrd_disp.h"
#include "mrd_eeprom.h"
#include "mrd_ether.h"
#include "mrd_move.h"
#include "mrd_sd.h"
#include "mrd_servo.h"
#include "mrd_util.h"
#include "mrd_wifi.h"
#include "mrd_wire0.h"

MERIDIANFLOW::Meridian mrd;

// サーボ系統ごとの半二重UARTバスと, その上で動くプロトコル層
MrdHalfDuplexUart sv_bus_l(&Serial1, PIN_RX_L, PIN_TX_L, PIN_EN_L, cfg.servo_echo_l, cfg.servo_timeout_l, cfg.servo_tx_release);
MrdHalfDuplexUart sv_bus_r(&Serial2, PIN_RX_R, PIN_TX_R, PIN_EN_R, cfg.servo_echo_r, cfg.servo_timeout_r, cfg.servo_tx_release);
// 通信速度は起動時に mrd_settings_setup() で設定値(EEPROM を含む)から決める
IcsHalfDuplexClass ics_l(sv_bus_l, 0);
IcsHalfDuplexClass ics_r(sv_bus_r, 0);
FeetechHalfDuplexClass ftc_l(sv_bus_l, 0);
FeetechHalfDuplexClass ftc_r(sv_bus_r, 0);
Dynamixel2HalfDuplexClass dxl2_l(sv_bus_l, 0);
Dynamixel2HalfDuplexClass dxl2_r(sv_bus_r, 0);
FutabaHalfDuplexClass ftb_l(sv_bus_l, 0);
FutabaHalfDuplexClass ftb_r(sv_bus_r, 0);
XbusHalfDuplexClass xbus_l(sv_bus_l, 0);
XbusHalfDuplexClass xbus_r(sv_bus_r, 0);
HitecHalfDuplexClass htc_l(sv_bus_l, 0);
HitecHalfDuplexClass htc_r(sv_bus_r, 0);
PmxHalfDuplexClass pmx_l(sv_bus_l, 0);
PmxHalfDuplexClass pmx_r(sv_bus_r, 0);
Dynamixel1HalfDuplexClass dxl1_l(sv_bus_l, 0);
Dynamixel1HalfDuplexClass dxl1_r(sv_bus_r, 0);

// ハードウェアタイマーとカウンタ用変数の定義
hw_timer_t *timer = NULL;                              // ハードウェアタイマーの設定
volatile SemaphoreHandle_t timer_semaphore;            // ハードウェアタイマー用のセマフォ
portMUX_TYPE timer_mux = portMUX_INITIALIZER_UNLOCKED; // ハードウェアタイマー用のミューテックス
unsigned long count_frame = 0;                         // フレーム処理の完了時にカウントアップ
volatile unsigned long count_timer = 0;                // フレーム用タイマーのカウントアップ

// Ethernet送信先IP（事前パース）
IPAddress ether_send_ip(0, 0, 0, 0); // Ethernet送信先IP（初期化）

// PCからの受信の管理
unsigned long udp_valid_ms = 0;      // 最後に正しいMeridimを受信した時刻(ms. フェイルセーフの判定用)
int udp_self_frames = 0;             // 受信しないモード(MODE_UDP_RECEIVE 0)で, 起動後に脱力したフレーム数
const int UDP_SELF_LIMP_FRAMES = 50; // 受信しないモードで, 起動後に脱力して現在位置を読むフレーム数(全サーボを読み終える長さ)
float udp_self_cmd_l[IXL_MAX] = {0}; // 受信しないモードで, 前のフレームに送った指令値(UDP_SELF_SPEED の基準). L系統
float udp_self_cmd_r[IXR_MAX] = {0}; // 同 R系統

/// @brief count_timerを保護しつつ1ずつインクリメント
void IRAM_ATTR mrd_frame_timer_isr() {
  portENTER_CRITICAL_ISR(&timer_mux);
  count_timer++;
  portEXIT_CRITICAL_ISR(&timer_mux);
  BaseType_t woken_tmp = pdFALSE;
  xSemaphoreGiveFromISR(timer_semaphore, &woken_tmp); // セマフォを与える
  if (woken_tmp == pdTRUE) {
    portYIELD_FROM_ISR(); // 待機中のメインループを次のティックを待たずに再開させる
  }
}

//==================================================================================================
//  SETUP / LOOP から呼ぶ処理
//==================================================================================================

/// @brief Core0 のスレッドを開始する. 作れなければ(メモリ不足)表示して停止する.
void mrd_task_start(TaskFunction_t a_func, const char *a_name, uint32_t a_stack, void *a_arg, UBaseType_t a_priority,
                    TaskHandle_t *a_handle) {
  if (xTaskCreatePinnedToCore(a_func, a_name, a_stack, a_arg, a_priority, a_handle, 0) != pdPASS) {
    mrd_error_stop(mrd_pin_err_led(), "ERROR: Could not start the thread " + String(a_name) + " (out of memory).", Serial);
  }
}

/// @brief スロットの設定(config の配列. EEPROM の P1/P2 を使う場合はその値)をサーボパラメータに反映し, サーボの通信を開始する.
void mrd_servo_setup() {
  // サーボ値の初期設定
  sv.num_max = max(mrd_index_max_used_get(IXL_MT, IXL_MAX),  //
                   mrd_index_max_used_get(IXR_MT, IXR_MAX)); // サーボ処理回数
  for (int i = 0; i < sv.num_max; i++) {                     // configで設定した値を反映させる
    // UARTの系統で扱うのはPWM専用(種類1/11)以外のスロット. PWM専用スロットは指令値をそのまま返す([9-1])
    //  EEPROM の P1 でマウントを外したスロットも扱わない(EEPROM で新たにマウントすることはない)
    sv.ixl_mount[i] = (IXL_MT[i] != NOSERVO && !mrd_servo_is_pwm_type(IXL_MT[i]) && !cfg.ixl_unmount[i]);
    sv.ixr_mount[i] = (IXR_MT[i] != NOSERVO && !mrd_servo_is_pwm_type(IXR_MT[i]) && !cfg.ixr_unmount[i]);
    sv.ixl_type[i] = IXL_MT[i];
    sv.ixr_type[i] = IXR_MT[i];
    sv.ixl_id[i] = IXL_ID[i];
    sv.ixr_id[i] = IXR_ID[i];
    sv.ixl_cw[i] = IXL_CW[i];
    sv.ixr_cw[i] = IXR_CW[i];
    sv.ixl_trim[i] = IXL_TRIM[i];
    sv.ixr_trim[i] = IXR_TRIM[i];
  }

  // サーボUARTの通信速度の表示
  mrd_disp.servo_bps_2lines(cfg.servo_baudrate_l, cfg.servo_baudrate_r);

  // サーボ用UART設定
  mrd_servo_line_begin(L, cfg.mount_servo_type_l, cfg.mount_servo_type_l2, sv); // サーボモータの通信初期設定. Serial1
  mrd_servo_line_begin(R, cfg.mount_servo_type_r, cfg.mount_servo_type_r2, sv); // サーボモータの通信初期設定. Serial2
  mrd_disp.servo_protocol(L, cfg.mount_servo_type_l);                           // サーボプロトコルの表示
  mrd_disp.servo_protocol(L, mrd_servo_type2[L]);                               // 2種類目(混在時のみ)
  mrd_disp.servo_protocol(R, cfg.mount_servo_type_r);
  mrd_disp.servo_protocol(R, mrd_servo_type2[R]);
  if (cfg.mount_servo_type_l != NOSERVO) { // 送信エコー処理の有無の表示(サーボなしの系統は通信しないので表示しない)
    mrd_disp.servo_echo(L, sv_bus_l.echo());
  }
  if (cfg.mount_servo_type_r != NOSERVO) {
    mrd_disp.servo_echo(R, sv_bus_r.echo());
  }
  mrd_pwm_begin(); // PWMサーボ(本体ピン/PCA9685)の開始

  // マウントされたサーボIDの表示
  mrd_disp.servo_mounts_2lines(sv);
}

/// @brief 系統のプロトコル層の通信速度. 2種類目のサーボ(MOUNT_SERVO_TYPE_x2)の系列なら, その通信速度.
/// @param a_line 系統(L/R).
/// @param a_family プロトコル層のサーボの系列(mrd_servo_family の値).
unsigned long mrd_line_baud(UartLine a_line, int a_family) {
  const int type2_tmp = (a_line == L) ? cfg.mount_servo_type_l2 : cfg.mount_servo_type_r2;
  if (type2_tmp != NOSERVO && mrd_servo_family(type2_tmp) == a_family) {
    return (a_line == L) ? cfg.servo_baudrate_l2 : cfg.servo_baudrate_r2;
  }
  return (a_line == L) ? cfg.servo_baudrate_l : cfg.servo_baudrate_r;
}

/// @brief 起動の最初に設定値(cfg)を決める. EEPROM_LOAD で選んだページは EEPROM の値を使う(仕様書16章).
///        決めた値を, フラグ・表示の設定と, サーボのバス・プロトコル層に反映する.
///        シリアルの開始より前に行う(通信速度も設定値のため). 結果の表示は mrd_eeprom_boot_report() で行う.
void mrd_settings_setup() {
  mrd_eeprom_settings_boot(EEPROM_LOAD, EEPROM_SET, EEPROM_PROTECT);

  flg.udp_receive_mode = cfg.mode_udp_receive;
  flg.udp_send_mode = cfg.mode_udp_send;

  // サーボのバスとプロトコル層(グローバルの生成時は config の値なので, 設定し直す. 通信速度はここで決める)
  sv_bus_l.configure(cfg.servo_echo_l, cfg.servo_timeout_l, cfg.servo_tx_release);
  sv_bus_r.configure(cfg.servo_echo_r, cfg.servo_timeout_r, cfg.servo_tx_release);
  ics_l.set_baud(mrd_line_baud(L, KOICS3));
  ics_r.set_baud(mrd_line_baud(R, KOICS3));
  ftc_l.set_baud(mrd_line_baud(L, FTCSTS));
  ftc_r.set_baud(mrd_line_baud(R, FTCSTS));
  dxl2_l.set_baud(mrd_line_baud(L, DXL2));
  dxl2_r.set_baud(mrd_line_baud(R, DXL2));
  ftb_l.set_baud(mrd_line_baud(L, FTBRSX));
  ftb_r.set_baud(mrd_line_baud(R, FTBRSX));
  xbus_l.set_baud(mrd_line_baud(L, JRXBUS));
  xbus_r.set_baud(mrd_line_baud(R, JRXBUS));
  htc_l.set_baud(mrd_line_baud(L, HTCHSR));
  htc_r.set_baud(mrd_line_baud(R, HTCHSR));
  pmx_l.set_baud(mrd_line_baud(L, KOPMX));
  pmx_r.set_baud(mrd_line_baud(R, KOPMX));
  dxl1_l.set_baud(mrd_line_baud(L, DXL1));
  dxl1_r.set_baud(mrd_line_baud(R, DXL1));
}

/// @brief WiFi(MODE_ETHER 0)または有線LAN(MODE_ETHER 1)を開始する. 設定の誤りがあれば停止する.
void mrd_network_setup() {
  // WiFiの初期化と開始
  if (!cfg.mode_ether) { // MODE_ETHER = 0 ならWiFiの初期化
    mrd_disp.esp_wifi(cfg.wifi_ap_ssid);
    if (cfg.mode_fixed_ip) { // 固定IPを使用する場合はwifi.configの設定を使用する
      IPAddress fixed_ip_tmp = mrd_ip_address_parse(cfg.fixed_ip_addr, Serial);
      IPAddress fixed_gw_tmp = mrd_ip_address_parse(cfg.fixed_ip_gateway, Serial);
      IPAddress fixed_sb_tmp = mrd_ip_address_parse(cfg.fixed_ip_subnet, Serial);
      if (mrd_network_config_validate(fixed_ip_tmp, fixed_gw_tmp, fixed_sb_tmp, Serial)) { // IPチェック
        WiFi.config(fixed_ip_tmp, fixed_gw_tmp, fixed_sb_tmp);                             // 固定IPを設定
        Serial.println("FIXEDIP****");
      } else { // IPのパースが失敗なら停止
        mrd_error_stop(mrd_pin_err_led(), "Please Check '#define FIXED_IP_ADDR, FIXED_IP_GATEWAY, FIXED_IP_SUBNET' in 'keys.h'", Serial);
      }
    }
    if (mrd_wifi_init(udp, cfg.wifi_ap_ssid, cfg.wifi_ap_pass, Serial)) {      // wifiの初期化
      mrd_disp.esp_ip(cfg.mode_fixed_ip, cfg.wifi_send_ip, cfg.fixed_ip_addr); // wifiIPの表示
    }

  } else { // MODE_ETHER = 1 ならEthernet初期化

    byte ether_mac_tmp[6];
    if (mrd_ether_mac_address_parse(cfg.ether_mac, ether_mac_tmp)) {

      if (mrd_ether_init(udp_et, PIN_CHIPSELECT_LAN, ether_mac_tmp, Serial)) {
        // Ethernet送信先IPの事前パース
        ether_send_ip = mrd_ip_address_parse(cfg.ether_gateway, Serial);

        if (ether_send_ip == IPAddress(0, 0, 0, 0)) {
          // エラー状態でシステム停止（LEDで視覚的に通知）
          mrd_error_stop(mrd_pin_err_led(), "ERROR: Ethernet initialization failed. Fix ETHER_GATEWAY and restart", Serial);
        }
      } else {
        mrd_error_stop(mrd_pin_err_led(), "ERROR: Ethernet initialization failed. Check Ethernet hardware/config.", Serial);
      }
    } else {
      Serial.print("ERROR: Failed to parse MAC address ");
      Serial.println(cfg.ether_mac);
      mrd_error_stop(mrd_pin_err_led(), "Please check '#define ETHER_MAC' in 'keys.h'", Serial);
    }
  }
}

/// @brief BlueRetro(Wii extension)を読むスレッドを開始する. PAD_BLUERETRO_I2C 0 なら I2C0, 1 なら I2C1.
///        I2C1 のピンを本体PWM(PWM_PIN_MODE 1)に使う場合は開始しない.
void mrd_blueretro_setup() {
  const bool i2c1_tmp = (cfg.pad_blueretro_i2c == 1);
  const int sda_tmp = i2c1_tmp ? PIN_I2C1_SDA : PIN_I2C0_SDA;
  const int scl_tmp = i2c1_tmp ? PIN_I2C1_SCL : PIN_I2C0_SCL;
  if (i2c1_tmp && cfg.pwm_pin_mode == 1 &&
      (sda_tmp == PIN_PWM_0 || sda_tmp == PIN_PWM_1 || scl_tmp == PIN_PWM_0 || scl_tmp == PIN_PWM_1)) {
    Serial.println("BlueRetro: I2C1 pins are used by the body PWM (PWM_PIN_MODE 1). Not started.");
    return;
  }
  TwoWire &wire_tmp = i2c1_tmp ? Wire1 : Wire;
  wire_tmp.begin(sda_tmp, scl_tmp, i2c1_tmp ? cfg.i2c1_speed : cfg.i2c0_speed); // IMU / PCA9685 で開始済みでも同じ設定
  mrd_task_start(mrd_blueretro_task, "blueretro", 2048, &wire_tmp, 5, &thp[2]);
  Serial.println("BlueRetro: I2C" + String(i2c1_tmp ? 1 : 0) + " SDA GPIO" + String(sda_tmp) + ", SCL GPIO" + String(scl_tmp) +
                 " (Wii extension 0x52).");
}

/// @brief Meridimを送信する(WiFiまたは有線LAN).
void mrd_udp_send() {
  if (!cfg.mode_ether) { // 0ならwifi通信
    mrd_wifi_udp_send(s_udp_meridim.bval, MRDM_BYTE, udp);
  } else { // 1なら有線LAN通信. 送信先は起動時にパース済み
    mrd_ether_udp_send(s_udp_meridim.bval, MRDM_BYTE, udp_et, ether_send_ip, cfg.udp_send_port);
  }
}

/// @brief Meridimを受信するまで待つ(最大 UDP_TIMEOUT_US. UDP_POLL_US ごとに確認).
/// @return 受信できればtrue. タイムアウトならfalse.
bool mrd_udp_receive_wait() {
  const unsigned long wait_start_us_tmp = micros();
  while (true) {
    const bool rcvd_tmp = !cfg.mode_ether ? mrd_wifi_udp_receive(r_udp_meridim.bval, MRDM_BYTE, udp)      // wifi
                                          : mrd_ether_udp_receive(r_udp_meridim.bval, MRDM_BYTE, udp_et); // 有線LAN
    if (rcvd_tmp) {
      return true;
    }
    if (micros() - wait_start_us_tmp >= cfg.udp_timeout_us) {
      if (cfg.monitor_udp_timeout && millis() > cfg.monitor_suppress_duration) { // 起動直後はエラー表示を抑制
        Serial.println("UDP timeout");
      }
      return false;
    }
    delayMicroseconds(cfg.udp_poll_us); // 受信確認の間隔
  }
}

/// @brief PCから受信しない場合(MODE_UDP_RECEIVE 0)に, 受信データの代わりの指令をボードが作る.
///        起動後 UDP_SELF_LIMP_FRAMES フレームは全サーボを脱力して現在位置を読み, 以後は全スロットをトルクONにして
///        マスターコマンド3(前回の指令値を保持)で動かす. 目標はESP32内の計算([7-2])で決める.
///        連番は予想どおりの値にし, 受信の確認([2-2]〜[2-6])は通常どおり通す.
void mrd_meridim_self_make() {
  short cmd_tmp = 1; // トルクON
  if (udp_self_frames < UDP_SELF_LIMP_FRAMES) {
    udp_self_frames++;
    cmd_tmp = 0; // 脱力して現在位置を読む
  } else if (udp_self_frames == UDP_SELF_LIMP_FRAMES) {
    udp_self_frames++;
    for (int i = 0; i < sv.num_max; i++) { // トルクONの基準を, 読んだ現在位置にする
      sv.ixl_cmd_past[i] = sv.ixl_tgt[i];
      sv.ixr_cmd_past[i] = sv.ixr_tgt[i];
      udp_self_cmd_l[i] = sv.ixl_tgt[i];
      udp_self_cmd_r[i] = sv.ixr_tgt[i];
    }
  }
  r_udp_meridim.sval[MRD_MASTER] = 3; // ESP32内の計算を優先
  for (int i = 0; i < MRD_SERVO_SLOTS; i++) {
    r_udp_meridim.sval[MRD_L_ORIGIDX + i * 2] = cmd_tmp;
    r_udp_meridim.sval[MRD_R_ORIGIDX + i * 2] = cmd_tmp;
  }
  r_udp_meridim.usval[MRD_SEQ] = mrd_seq_num_predict(mrdsq.r_expect); // 連番を予想どおりにする
  r_udp_meridim.sval[MRD_CKSM] = mrd.cksm_val(r_udp_meridim.sval, MRDM_LEN);
}

/// @brief 受信しないモード(MODE_UDP_RECEIVE 0)のトルクON後に, 指令値の1フレームの変化量を UDP_SELF_SPEED に制限する.
///        [7-2] で決めた目標へ, 前のフレームに送った指令値から一定の速さで近づける.
///        目標(cmd_past)は書き換えないので, 一度だけ目標を決めても, 届くまで近づき続ける.
void mrd_self_speed_limit() {
  if (flg.udp_receive_mode || udp_self_frames <= UDP_SELF_LIMP_FRAMES) {
    return;
  }
  const float step_tmp = cfg.udp_self_speed * cfg.frame_duration / 1000.0f; // 1フレームあたりの最大変化量(degree)
  for (int i = 0; i < sv.num_max; i++) {
    sv.ixl_tgt[i] = mrd_step_limit(sv.ixl_tgt[i], udp_self_cmd_l[i], step_tmp);
    sv.ixr_tgt[i] = mrd_step_limit(sv.ixr_tgt[i], udp_self_cmd_r[i], step_tmp);
    udp_self_cmd_l[i] = sv.ixl_tgt[i];
    udp_self_cmd_r[i] = sv.ixr_tgt[i];
  }
}

/// @brief 受信したMeridimのチェックサムとシーケンス番号を確認し, 送信配列に転写してエラービットを更新する.
///        受信できなかったフレームとチェックサム不正のフレームは, どちらも受信エラーとして扱い,
///        最後に正しく受信したMeridim(r_udp_meridim_last)を使う.
/// @param a_udp_lost このフレームは受信できなかったか.
/// @return 正しいMeridimを受信できたか(受信あり, チェックサム正常). 連番の飛びは問わない.
bool mrd_meridim_received_check(bool a_udp_lost) {
  // @[2-2] チェックサムを確認し, 正しければ最後に正しく受信したMeridimとして保持する
  const bool valid_tmp = !a_udp_lost && mrd.cksm_rslt(r_udp_meridim.sval, MRDM_LEN);
  if (valid_tmp) {
    mrd_disp.flow("CsOK", cfg.monitor_flow); // デバグ用フロー表示
    memcpy(r_udp_meridim_last.bval, r_udp_meridim.bval, MRDM_LEN * 2);
  } else if (!a_udp_lost) {
    mrd_disp.flow("CsErr*", cfg.monitor_flow); // デバグ用フロー表示
  }

  // @[2-3] 最後に正しく受信したMeridimを UDP送信配列に転写
  memcpy(s_udp_meridim.bval, r_udp_meridim_last.bval, MRDM_LEN * 2);

  // @[2-4] エラービット14番(ESP32のPCからのUDP受信エラー検出): 正しく受信できなければアゲる
  if (valid_tmp) {
    mrd_bit16_clear(s_udp_meridim.usval[MRD_ERR], ERRBIT_14_PC_ESP);
  } else {
    mrd_bit16_set(s_udp_meridim.usval[MRD_ERR], ERRBIT_14_PC_ESP);
    err.pc_esp++;
  }

  // @[2-5] シーケンス番号チェック
  //        正しく受信できなかったフレームは判定しない(受信エラーとして数え済み. 予想値も進めない).
  //        Meridim受信成功フラグはサゲ, 前回のデータでマスターコマンドを再実行しない.
  if (!valid_tmp) {
    mrd_bit16_clear(s_udp_meridim.usval[MRD_ERR], ERRBIT_10_UDP_ESP_SKIP);
    flg.meridim_rcvd = false; // Meridim受信成功フラグをサゲる.
  } else {
    mrdsq.r_expect = mrd_seq_num_predict(mrdsq.r_expect); // シーケンス番号予想値の生成

    // @[2-6] シーケンス番号のシリアルモニタ表示
    mrd_disp.seq_number(mrdsq.r_expect, s_udp_meridim.usval[MRD_SEQ], cfg.monitor_seq);

    if (mrd.seq_compare_nums(mrdsq.r_expect, int(s_udp_meridim.usval[MRD_SEQ]))) {

      // エラービット10番[ESP受信のスキップ検出]をサゲる
      mrd_bit16_clear(s_udp_meridim.usval[MRD_ERR], ERRBIT_10_UDP_ESP_SKIP);
      flg.meridim_rcvd = true; // Meridim受信成功フラグをアゲる.

    } else {                                              // 受信シーケンス番号の値が予想と違ったら
      mrdsq.r_expect = int(s_udp_meridim.usval[MRD_SEQ]); // 現在の受信値を予想結果としてキープ

      // エラービット10番[ESP受信のスキップ検出]をアゲる
      mrd_bit16_set(s_udp_meridim.usval[MRD_ERR], ERRBIT_10_UDP_ESP_SKIP);

      err.esp_skip++;
      flg.meridim_rcvd = false; // Meridim受信成功フラグをサゲる.
    }
  }
  return valid_tmp;
}

/// @brief PCからの受信が UDP_FAILSAFE_MS 以上途絶えたら, 全スロットのコマンドを0(脱力)にする(フェイルセーフ).
///        正しいMeridimを受信すると, 受信したコマンドに従う通常の動作に戻る. 入るときと戻るときに1回ずつ表示する.
/// @param a_valid このフレームで正しいMeridim(受信あり, チェックサム正常)を受信できたか.
void mrd_udp_failsafe(bool a_valid) {
  if (a_valid) {
    udp_valid_ms = millis();
    if (flg.udp_failsafe) {
      flg.udp_failsafe = false;
      Serial.println("UDP restored: servos follow the PC commands again.");
    }
    return;
  }
  if (cfg.udp_failsafe_ms <= 0 || millis() - udp_valid_ms < (unsigned long)cfg.udp_failsafe_ms) {
    return;
  }
  if (!flg.udp_failsafe) {
    flg.udp_failsafe = true;
    Serial.println("UDP lost for " + String(cfg.udp_failsafe_ms) + " ms: all servos torque off (failsafe).");
  }
  mrd_servo_all_off(s_udp_meridim, false);
}

/// @brief 受信した目標値から, 各サーボの今回の指令値(tgt)を求める.
void mrd_servo_targets_compute() {
  // @[7-1] 前回読み込んだサーボ位置を保持し, 受信した目標値をサーボ配列に書き込む.
  //        目標値は Meridim[19](MRD_MOTION_FRAMES)のフレーム数をかけて直線で補間する(0,1は即時).
  //        1フレームあたりの指令値の変化量はSERVO_STEP_MAX[degree]に制限する.
  //        制限の基準は前回の指令値. 脱力中(コマンド0)は読み込んだ実角度を基準にし,
  //        トルクオン時に現在の姿勢から動き出すようにする.
  //        通常以外のスロット(パラメータ系コマンド, 車輪, スルー)は, VALが角度ではないため前回の値を保持する.
  //        マスターコマンド3(ESP32内の計算を優先)のときは, PCの目標値を使わず前回の指令値を保持する([7-2]で計算する).
  const int frames_tmp = constrain(int(s_udp_meridim.sval[MRD_MOTION_FRAMES]), 0, cfg.servo_motion_frames_max);
  const bool board_priority_tmp = (s_udp_meridim.sval[MRD_MASTER] == 3);
  for (int line = 0; line < 2; line++) { // 0:L系統, 1:R系統
    float *tgt_tmp = (line == 0) ? sv.ixl_tgt : sv.ixr_tgt;
    float *tgt_past_tmp = (line == 0) ? sv.ixl_tgt_past : sv.ixr_tgt_past;
    float *cmd_past_tmp = (line == 0) ? sv.ixl_cmd_past : sv.ixr_cmd_past;
    const int orig_tmp = (line == 0) ? MRD_L_ORIGIDX : MRD_R_ORIGIDX;
    for (int i = 0; i < sv.num_max; i++) {
      const short cmd_tmp = s_udp_meridim.sval[orig_tmp + i * 2];
      const int16_t val_tmp = s_udp_meridim.sval[orig_tmp + 1 + i * 2];
      MrdMotion &mot_tmp = mrd_servo_motion[line][i];
      tgt_past_tmp[i] = tgt_tmp[i]; // 前回のdegreeをキープ
      if (mrd_servo_slot_mode(cmd_tmp) != MRD_SLOT_NORMAL) {
        mrd_servo_motion_reset(mot_tmp);
        continue;
      }
      if (mrd_servo_cmd(cmd_tmp) != 1) { // 脱力中: 読み込んだ実角度を基準にする
        mrd_servo_motion_reset(mot_tmp);
        tgt_tmp[i] = mrd_step_limit(val_tmp * 0.01f, tgt_past_tmp[i], cfg.servo_step_max);
      } else if (board_priority_tmp) { // マスターコマンド3: 前回の指令値のまま([7-2]で計算する)
        mrd_servo_motion_reset(mot_tmp);
        tgt_tmp[i] = cmd_past_tmp[i];
      } else { // トルクON: 補間した目標へ, 前回の指令値から SERVO_STEP_MAX 以内で近づける
        const float goal_tmp = (cfg.servo_motion_frames_max > 0)
                                   ? mrd_servo_motion_target(mot_tmp, val_tmp, cmd_past_tmp[i], frames_tmp)
                                   : val_tmp * 0.01f;
        tgt_tmp[i] = mrd_step_limit(goal_tmp, cmd_past_tmp[i], cfg.servo_step_max);
      }
      cmd_past_tmp[i] = tgt_tmp[i]; // 今回の指令値を次回の基準としてキープ
    }
  }
}

/// @brief 各スロットの返信値(現在位置, 指令値, パラメータ系コマンドの結果)を送信配列に格納する.
void mrd_servo_values_reply() {
  // @[9-1] サーボIDごとにの現在位置もしくは計算結果を配列に格納
  //        PWM専用スロット(種類1/11)と戻り値のないサーボ(PWM, Hitec, XBUS)は指令値を返す.
  //        マウントしていないスロットは, SERVO_UNMOUNT_REPLY が0ならコマンド・角度とも0を返し,
  //        1なら受信した値をそのまま返す(s_udp_meridimは受信データのコピーなので書き換えない).
  //        マウントしていないスロット: 系統のサーボの種類が0, スロットの種類が0, 起動時の検査でマウントを
  //        外したもの(sv.ixl_mount/ixr_mount が0), num_max以降.
  //        パラメータ系コマンド(コマンドの下位8bitが3以上)のスロットは, その結果を返す(エラーならbit14).
  for (int i = 0; i < MRD_SERVO_SLOTS; i++) {
    mrd_servo_slot_reply(s_udp_meridim, MRD_L_ORIGIDX, i, i < IXL_MAX ? IXL_MT[i] : NOSERVO,
                         cfg.mount_servo_type_l != NOSERVO && i < sv.num_max && i < IXL_MAX && sv.ixl_mount[i],
                         i < IXL_MAX ? &sv.ixl_tgt[i] : nullptr, &mrd_servo_param[L][i]);
    mrd_servo_slot_reply(s_udp_meridim, MRD_R_ORIGIDX, i, i < IXR_MAX ? IXR_MT[i] : NOSERVO,
                         cfg.mount_servo_type_r != NOSERVO && i < sv.num_max && i < IXR_MAX && sv.ixr_mount[i],
                         i < IXR_MAX ? &sv.ixr_tgt[i] : nullptr, &mrd_servo_param[R][i]);
  }
}

/// @brief 今のフレームの処理が, 次のフレームの時刻までに終わらなかったか.
bool mrd_frame_is_late() {
  portENTER_CRITICAL(&timer_mux);
  const bool late_tmp = (count_timer >= count_frame);
  portEXIT_CRITICAL(&timer_mux);
  return late_tmp;
}

/// @brief タイマー(count_timer)がフレーム数(count_frame)に追いつくまで待つ.
///        タイマー割り込みでセマフォが与えられるまでブロックし, その間CPUを空ける.
///        フレームタイマーのリセットを指示されていれば(MCMD_FRAMETIMER_RESET), 遅れの累積を捨てる.
void mrd_frame_next_wait() {
  while (true) {
    portENTER_CRITICAL(&timer_mux);
    const unsigned long current_count_timer_tmp = count_timer; // ハードウェアタイマーの値を読む
    portEXIT_CRITICAL(&timer_mux);
    if (current_count_timer_tmp >= count_frame) {
      break;
    }
    xSemaphoreTake(timer_semaphore, portMAX_DELAY);
  }
  if (flg.count_frame_reset) {
    portENTER_CRITICAL(&timer_mux);
    count_frame = count_timer;
    portEXIT_CRITICAL(&timer_mux);
    flg.count_frame_reset = false; // リセットは1回だけ行う
  }
}

//==================================================================================================
//  SETUP
//==================================================================================================
void setup() {

  // 設定値を決める(EEPROM_LOAD で選んだページは EEPROM の値)
  mrd_settings_setup();

  // BT接続確認用LED設定
  if (mrd_pin_led_bt() >= 0) { // ピン26をPWM等に使う場合はLEDなし
    pinMode(mrd_pin_led_bt(), OUTPUT);
  }
  mrd_led_write(mrd_pin_led_bt(), HIGH);

  // シリアルモニターの設定(ESP32ではシリアルの接続を検出できないため, 接続は待たない)
  Serial.begin(cfg.serial_pc_bps);

  // ピンモードの設定
  if (mrd_pin_err_led() >= 0) { // エラー通知用LED(ピン25をPWM等に使う場合はなし)
    pinMode(mrd_pin_err_led(), OUTPUT);
  }

  // ボード搭載のコンデンサの充電時間として待機
  mrd_disp.charging(cfg.charge_time);

  // 起動メッセージの表示(バージョン, PC-USB,SPI0,i2c0のスピード)
  mrd_disp.hello_lite_esp(VERSION, cfg.serial_pc_bps, SPI0_SPEED, cfg.i2c0_speed);

  // EEPROMと設定値の表示(どのページが EEPROM の値か, 書き込みの結果, 内容のダンプ)
  mrd_eeprom_boot_report(Serial);
  mrd_eeprom_dump_at_boot(EEPROM_DUMP, EEPROM_STYLE, Serial);

  // サーボの設定値の反映と通信の開始
  mrd_servo_setup();

  // SDカードの初期設定とチェック
  mrd_sd_init(cfg.mount_sd, PIN_CHIPSELECT_SD);
  mrd_sd_check(cfg.mount_sd, PIN_CHIPSELECT_SD, cfg.check_sd_rw);

  // I2Cの初期化と開始
  bool imuahrs_ok_tmp = mrd_wire0_setup(ImuAhrsType(cfg.mount_imuahrs), cfg.i2c0_speed, ahrs, PIN_I2C0_SDA, PIN_I2C0_SCL);

  // I2Cスレッドの開始(センサの初期化に成功した場合のみ)
  if (imuahrs_ok_tmp && cfg.mount_imuahrs == BNO055_AHRS) {
    mrd_task_start(mrd_wire0_bno055_task, "wire0_bno055", 4096, NULL, 2, &thp[0]);
    Serial.println("Core0 thread for BNO055 start.");
    delay(10);
  } else if (imuahrs_ok_tmp && cfg.mount_imuahrs == MPU6050_IMU) {
    mrd_task_start(mrd_wire0_mpu6050_task, "wire0_mpu6050", 4096, NULL, 2, &thp[0]);
    Serial.println("Core0 thread for MPU6050 start.");
    delay(10);
  }

  // WiFiまたは有線LANの開始
  mrd_network_setup();

  // コントロールパッドの種類を表示
  mrd_disp.mounted_pad(cfg.mount_pad);

  // パッドの受信の開始
  if (cfg.mount_pad == BLUERETRO) { // BlueRetro: I2C0(Wire. IMUと同じバス)かI2C1(Wire1)で読むスレッドの開始
    mrd_blueretro_setup();
  }
  if (cfg.mount_pad == SBDBT) { // SBDBT の受信の開始(PIN_PAD_RX の信号の変化を割り込みで記録する)
    mrd_sbdbt_begin(PIN_PAD_RX, cfg.pad_sbdbt_bps);
    Serial.println("SBDBT: RX GPIO" + String(PIN_PAD_RX) + ", " + String(cfg.pad_sbdbt_bps) + " bps (RCB3 format).");
  }
  if (cfg.mount_pad == WIIMOTE || cfg.mount_pad == WIIMOTE_C) { // Bluetooth用スレッドの開始
    mrd_bt_setup(cfg.mount_pad, cfg.pad_init_timeout, wiimote, mrd_pin_led_bt(), Serial);
    mrd_task_start(mrd_bt_wiimote_task, "bt_wiimote", 4096, NULL, 5, &thp[2]);
  }

  // UDP開始用ダミーデータの生成
  r_udp_meridim.sval[MRD_MASTER] = 90;
  r_udp_meridim.sval[MRD_CKSM] = mrd.cksm_val(r_udp_meridim.sval, MRDM_LEN);
  s_udp_meridim = r_udp_meridim;
  r_udp_meridim_last = r_udp_meridim; // 最初の正しい受信までは, このダミー(全サーボ脱力)を使う

  // タイマーの設定
  timer_semaphore = xSemaphoreCreateBinary(); // セマフォの作成
  if (timer_semaphore == NULL) {
    mrd_error_stop(mrd_pin_err_led(), "ERROR: Could not create the frame timer semaphore (out of memory).", Serial);
  }
  timer = timerBegin(0, 80, true); // タイマーの設定(1つ目のタイマーを使用, 分周比80)

  timerAttachInterrupt(timer, &mrd_frame_timer_isr, true); // mrd_frame_timer_isr関数をタイマーの割り込みに登録
  timerAlarmWrite(timer, cfg.frame_duration * 1000, true); // タイマーを1フレームごとにトリガー
  timerAlarmEnable(timer);                                 // タイマーを開始

  // 開始メッセージ
  mrd_disp.flow_start_lite_esp();
  if (cfg.mode_esp32_standalone) {
    Serial.println("CAUTION: This is ESP32 Standalone Mode. Servo communication will be ignored.");
  }

  // タイマーの初期化
  count_frame = 0;
  portENTER_CRITICAL(&timer_mux);
  count_timer = 0;
  portEXIT_CRITICAL(&timer_mux);
  udp_valid_ms = millis(); // フェイルセーフの判定は開始時から数える
}

//==================================================================================================
// MAIN LOOP
//==================================================================================================
void loop() {
  const unsigned long loop_start_us_tmp = micros(); // フレーム処理時間の計測用
  unsigned long servo_us_tmp = 0;                   // サーボ通信時間の計測用

  //------------------------------------------------------------------------------------
  //  [ 1 ] UDP送信
  //------------------------------------------------------------------------------------
  mrd_disp.flow("[1]", cfg.monitor_flow); // デバグ用フロー表示

  // @[1-1] UDP送信の実行
  if (flg.udp_send_mode) { // UDPの送信実施フラグの確認(モード確認)
    mrd_udp_send();
  }

  //------------------------------------------------------------------------------------
  //  [ 2 ] UDP受信
  //------------------------------------------------------------------------------------
  mrd_disp.flow("[2]", cfg.monitor_flow); // デバグ用フロー表示

  // @[2-1] UDPの受信待ち. 受信できなかった(タイムアウト)場合は前回の受信データをそのまま使い,
  //        PCからの受信エラー(エラービット14番)として扱う. シーケンス番号は進めない.
  //        受信しないモードでは, 受信データの代わりにボードが指令を作る.
  bool udp_lost_tmp = false;
  if (flg.udp_receive_mode) { // UDPの受信実施フラグの確認(モード確認)
    udp_lost_tmp = !mrd_udp_receive_wait();
  } else {
    mrd_meridim_self_make();
  }

  // @[2-2]〜[2-6] チェックサムとシーケンス番号を確認し, 受信データを送信配列に転写する
  const bool udp_valid_tmp = mrd_meridim_received_check(udp_lost_tmp);

  // @[2-7] 受信が UDP_FAILSAFE_MS 以上途絶えていたら全サーボを脱力する(フェイルセーフ)
  mrd_udp_failsafe(udp_valid_tmp);

  //------------------------------------------------------------------------------------
  //  [ 3 ] MasterCommand group1 の処理
  //------------------------------------------------------------------------------------
  mrd_disp.flow("[3]", cfg.monitor_flow); // デバグ用フロー表示

  // @[3-1] MasterCommand group1 の処理
  mrd_command_group1_execute(s_udp_meridim, flg.meridim_rcvd, sv, Serial);

  //------------------------------------------------------------------------------------
  //  [ 4 ] センサー類読み取り
  //------------------------------------------------------------------------------------
  mrd_disp.flow("[4]", cfg.monitor_flow); // デバグ用フロー表示

  // @[4-1] センサ値のMeridimへの転記
  mrd_meriput90_ahrs(s_udp_meridim, ahrs.read, cfg.mount_imuahrs); // BNO055_AHRS, MPU6050_IMU

  //------------------------------------------------------------------------------------
  //  [ 5 ] リモコンの読み取り
  //------------------------------------------------------------------------------------
  mrd_disp.flow("[5]", cfg.monitor_flow); // デバグ用フロー表示

  // @[5-1] リモコンデータの書き込み
  if (cfg.mount_pad > 0) { // リモコンがマウントされていれば

    // リモコンデータの読み込み
    pad_array.ui64val = mrd_pad_read(PadType(cfg.mount_pad));

    // リモコンの値をmeridimに格納する
    mrd_meriput90_pad(s_udp_meridim, pad_array, cfg.pad_button_marge);
  }

  //------------------------------------------------------------------------------------
  //  [ 6 ] MasterCommand group2 の処理
  //------------------------------------------------------------------------------------
  mrd_disp.flow("[6]", cfg.monitor_flow); // デバグ用フロー表示

  // @[6-1] MasterCommand group2 の処理
  mrd_command_group2_execute(s_udp_meridim, flg.meridim_rcvd, sv, Serial);

  //------------------------------------------------------------------------------------
  //  [ 7 ] ESP32内部で位置制御する場合の処理
  //------------------------------------------------------------------------------------
  mrd_disp.flow("[7]", cfg.monitor_flow); // デバグ用フロー表示

  // @[7-1] 受信した目標値からサーボの指令値を求める(補間と変化量の制限)
  mrd_servo_targets_compute();

  // @[7-2] ESP32による次回動作の計算
  // 以下はリモコンの左十字キー左右でL系統0番サーボ(首部)を30度左右にふるサンプル
  //        指令値の変化量は[7-1]と同じく, 前回の指令値からSERVO_STEP_MAX以内に制限する.
  // ボタンは16bitのビット列のため符号なし(usval)で比べる(PAD_bL = 32768 は short に入らない)
  if (s_udp_meridim.usval[MRD_PAD_BUTTONS] == PAD_RIGHT || s_udp_meridim.usval[MRD_PAD_BUTTONS] == PAD_LEFT) {
    const float goal_tmp = (s_udp_meridim.usval[MRD_PAD_BUTTONS] == PAD_RIGHT) ? -30.0f : 30.0f; // -30度, +30度
    sv.ixl_tgt[0] = mrd_step_limit(goal_tmp, sv.ixl_tgt_past[0], cfg.servo_step_max);
    sv.ixl_cmd_past[0] = sv.ixl_tgt[0]; // 今回の指令値を次回の基準としてキープ
  }

  // @[7-3] 受信しないモードでは, 指令値の変化を UDP_SELF_SPEED の速さに制限する
  mrd_self_speed_limit();

  // @[7-4] 各種処理

  //------------------------------------------------------------------------------------
  //  [ 8 ] サーボ動作の実行
  //------------------------------------------------------------------------------------
  mrd_disp.flow("[8]", cfg.monitor_flow); // デバグ用フロー表示

  // @[8-1] サーボ受信値の処理
  if (!cfg.mode_esp32_standalone) { // サーボ処理を行うかどうか
    const unsigned long servo_start_us_tmp = micros();
    mrd_servo_drive_lite(s_udp_meridim, cfg.mount_servo_type_l, cfg.mount_servo_type_r, sv); // サーボ動作を実行する
    servo_us_tmp = micros() - servo_start_us_tmp;
  } else {
    // ボード単体動作モードの場合はサーボ処理をせずL0番サーボ値として+-30度のサインカーブ値を返す
    tmr.count_loop = (tmr.count_loop + tmr.count_loop_dlt) % (tmr.count_loop_max + 1); // 1フレームごとに進める
    sv.ixl_tgt[0] = sinf(float(tmr.count_loop) * (float(M_PI) / 180.0f)) * 30.0f;
  }

  //------------------------------------------------------------------------------------
  //  [ 9 ] サーボ受信値の処理
  //------------------------------------------------------------------------------------
  mrd_disp.flow("[9]", cfg.monitor_flow); // デバグ用フロー表示

  // @[9-1] サーボごとの現在位置(または指令値, パラメータ系コマンドの結果)を送信配列に格納
  mrd_servo_values_reply();

  //------------------------------------------------------------------------------------
  //  [ 10 ] エラーリポートの作成
  //------------------------------------------------------------------------------------
  mrd_disp.flow("[10]", cfg.monitor_flow); // デバグ用フロー表示

  // @[10-1] エラーリポートの表示
  mrd_disp.all_err(cfg.monitor_err_all, err);

  //------------------------------------------------------------------------------------
  //  [ 11 ] MasterCommand group3 の処理
  //------------------------------------------------------------------------------------
  mrd_disp.flow("[11]", cfg.monitor_flow); // デバグ用フロー表示

  mrd_command_group3_execute(s_udp_meridim, flg.meridim_rcvd, sv, Serial);

  //------------------------------------------------------------------------------------
  //  [ 12 ] UDP送信信号作成
  //------------------------------------------------------------------------------------
  mrd_disp.flow("[12]", cfg.monitor_flow); // デバグ用フロー表示

  // @[12-1] フレームスキップ検出用のカウントをカウントアップして送信用に格納
  mrdsq.s_increment = mrd.seq_increase_num(mrdsq.s_increment);
  s_udp_meridim.usval[MRD_SEQ] = mrdsq.s_increment;

  // @[12-2] エラーが出たサーボのインデックス番号を格納
  s_udp_meridim.ubval[MRD_ERR_l] = mrd_servo_errcode_make_lite(sv);

  // @[12-3] チェックサムを計算して格納
  s_udp_meridim.sval[MRD_CKSM] = mrd.cksm_val(s_udp_meridim.sval, MRDM_LEN);

  //------------------------------------------------------------------------------------
  //   [ 13 ] フレーム終端処理
  //------------------------------------------------------------------------------------
  mrd_disp.flow("[13]", cfg.monitor_flow); // 動作チェック用シリアル表示

  // @[13-1] フレーム数を進め, 処理時間を表示する(処理の終了時点でタイマーが次のフレームに達していれば遅延)
  count_frame++;
  mrd_disp.frame_time(frame_mon, micros() - loop_start_us_tmp, servo_us_tmp, mrd_frame_is_late(), cfg.monitor_frame_delay, sv);

  // @[13-2] 次のフレームの時刻まで待つ(タイマー割り込みまでCPUを空ける)
  mrd_frame_next_wait();

  mrd_disp.flow("\n", cfg.monitor_flow); // 動作チェック用シリアル表示
}

#endif // __MERIDIAN_LITE_MAIN__
