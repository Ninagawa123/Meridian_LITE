#ifndef __MERIDIAN_LITE_MAIN__
#define __MERIDIAN_LITE_MAIN__

#define VERSION "Meridian_LITE_v1.1.2_2026_11.03" // バージョン表示

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
#include "main.h"
#include "config.h"
#include "keys.h"

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

// サーボ系統ごとの半二重UARTバス(3ステートバッファ版: 送信エコーなし)と, その上で動くプロトコル層.
// 通信速度は系統の設定(SERVO_BAUDRATE_L/R)で, 系統の種類(MOUNT_SERVO_TYPE_L/R)のプロトコル層だけが使われる
MrdHalfDuplexUart sv_bus_l(&Serial1, PIN_RX_L, PIN_TX_L, PIN_EN_L, MrdHalfDuplexUart::ECHO_OFF, SERVO_TIMEOUT_L);
MrdHalfDuplexUart sv_bus_r(&Serial2, PIN_RX_R, PIN_TX_R, PIN_EN_R, MrdHalfDuplexUart::ECHO_OFF, SERVO_TIMEOUT_R);
IcsHalfDuplexClass ics_l(sv_bus_l, SERVO_BAUDRATE_L);
IcsHalfDuplexClass ics_r(sv_bus_r, SERVO_BAUDRATE_R);
FeetechHalfDuplexClass ftc_l(sv_bus_l, SERVO_BAUDRATE_L);
FeetechHalfDuplexClass ftc_r(sv_bus_r, SERVO_BAUDRATE_R);
Dynamixel2HalfDuplexClass dxl2_l(sv_bus_l, SERVO_BAUDRATE_L);
Dynamixel2HalfDuplexClass dxl2_r(sv_bus_r, SERVO_BAUDRATE_R);
FutabaHalfDuplexClass ftb_l(sv_bus_l, SERVO_BAUDRATE_L);
FutabaHalfDuplexClass ftb_r(sv_bus_r, SERVO_BAUDRATE_R);
XbusHalfDuplexClass xbus_l(sv_bus_l, SERVO_BAUDRATE_L);
XbusHalfDuplexClass xbus_r(sv_bus_r, SERVO_BAUDRATE_R);
HitecHalfDuplexClass htc_l(sv_bus_l, SERVO_BAUDRATE_L);
HitecHalfDuplexClass htc_r(sv_bus_r, SERVO_BAUDRATE_R);
PmxHalfDuplexClass pmx_l(sv_bus_l, SERVO_BAUDRATE_L);
PmxHalfDuplexClass pmx_r(sv_bus_r, SERVO_BAUDRATE_R);
Dynamixel1HalfDuplexClass dxl1_l(sv_bus_l, SERVO_BAUDRATE_L);
Dynamixel1HalfDuplexClass dxl1_r(sv_bus_r, SERVO_BAUDRATE_R);

// ライブラリ導入
#include <Arduino.h>

// ハードウェアタイマーとカウンタ用変数の定義
hw_timer_t *timer = NULL;                              // ハードウェアタイマーの設定
volatile SemaphoreHandle_t timer_semaphore;            // ハードウェアタイマー用のセマフォ
portMUX_TYPE timer_mux = portMUX_INITIALIZER_UNLOCKED; // ハードウェアタイマー用のミューテックス
unsigned long count_frame = 0;                         // フレーム処理の完了時にカウントアップ
volatile unsigned long count_timer = 0;                // フレーム用タイマーのカウントアップ

// Ethernet送信先IP（事前パース）
IPAddress ether_send_ip(0, 0, 0, 0); // Ethernet送信先IP（初期化）

/// @brief count_timerを保護しつつ1ずつインクリメント
void IRAM_ATTR frame_timer() {
  portENTER_CRITICAL_ISR(&timer_mux);
  count_timer++;
  portEXIT_CRITICAL_ISR(&timer_mux);
  xSemaphoreGiveFromISR(timer_semaphore, NULL); // セマフォを与える
}

//==================================================================================================
//  SETUP
//==================================================================================================
void setup() {

  // BT接続確認用LED設定
  if (mrd_pin_led_bt() >= 0) { // ピン26をPWM等に使う場合はLEDなし
    pinMode(mrd_pin_led_bt(), OUTPUT);
  }
  mrd_led_write(mrd_pin_led_bt(), HIGH);

  // シリアルモニターの設定
  Serial.begin(SERIAL_PC_BPS);
  unsigned long start_time = millis();                             // シリアルモニターの確立待ち
  while (!Serial && (millis() - start_time < SERIAL_PC_TIMEOUT)) { // タイムアウトもチェック
    delay(1);
  }

  // ピンモードの設定
  if (mrd_pin_err_led() >= 0) { // エラー通知用LED(ピン25をPWM等に使う場合はなし)
    pinMode(mrd_pin_err_led(), OUTPUT);
  }

  // ボード搭載のコンデンサの充電時間として待機
  mrd_disp.charging(CHARGE_TIME);

  // 起動メッセージの表示(バージョン, PC-USB,SPI0,i2c0のスピード)
  mrd_disp.hello_lite_esp(VERSION, SERIAL_PC_BPS, SPI0_SPEED, I2C0_SPEED);

  // サーボ値の初期設定
  sv.num_max = max(mrd_max_used_index(IXL_MT, IXL_MAX),  //
                   mrd_max_used_index(IXR_MT, IXR_MAX)); // サーボ処理回数(使っているスロットの最大番号+1)
  for (int i = 0; i < sv.num_max; i++) {                 // configで設定した値を反映させる
    // UARTの系統で扱うのはPWM専用(種類1/11)以外のスロット. PWM専用スロットは指令値をそのまま返す([9-1])
    sv.ixl_mount[i] = (IXL_MT[i] != NOSERVO && !mrd_servo_is_pwm_type(IXL_MT[i]));
    sv.ixr_mount[i] = (IXR_MT[i] != NOSERVO && !mrd_servo_is_pwm_type(IXR_MT[i]));
    sv.ixl_type[i] = IXL_MT[i];
    sv.ixr_type[i] = IXR_MT[i];
    sv.ixl_id[i] = IXL_ID[i];
    sv.ixr_id[i] = IXR_ID[i];
    sv.ixl_cw[i] = IXL_CW[i];
    sv.ixr_cw[i] = IXR_CW[i];
    sv.ixl_trim[i] = IXL_TRIM[i];
    sv.ixr_trim[i] = IXR_TRIM[i];
  }

  // EEPROMの開始
  Serial.print("Initializing EEPROM... ");
  if (mrd_eeprom_init(EEPROM_SIZE)) { // EEPROMの初期化
    Serial.println("OK");
  } else {
    Serial.println("Failed");
  }

  // EEPROMにconfigのサーボ設定値を書き込む場合
  if (EEPROM_SET) {
    Serial.println("Set EEPROM data from config.");
    // 書き込みデータの作成と書き込み
    if (
        mrd_eeprom_write(mrd_eeprom_make_data_from_config(sv), EEPROM_PROTECT, Serial)) {
      Serial.println("Write EEPROM succeed.");
    } else {
      Serial.println("Write EEPROM failed.");
    };
  }

  // EEPROMからサーボ設定の内容を読み込んで反映する場合(サーボの通信開始で使うIDなどを決めるため, 通信開始の前に行う)
  if (EEPROM_LOAD) {
    mrd_eeprom_load_servosettings(sv, true, Serial);
  }

  // EEPROMの内容ダンプ表示をする場合
  mrd_eeprom_dump_at_boot(EEPROM_DUMP, EEPROM_STYLE, Serial); //

  // EEPROMのリードライトテスト
  // mrd_eeprom_write_read_check(mrd_eeprom_make_data_from_config(),
  //                             CHECK_EEPROM_RW, EEPROM_PROTECT, EEPROM_STYLE);

  // サーボUARTの通信速度の表示
  mrd_disp.servo_bps_2lines(SERVO_BAUDRATE_L, SERVO_BAUDRATE_R);

  // サーボ用UART設定(ボードなしの単体動作ではサーボ通信をしないので開始しない)
  if (!MODE_ESP32_STANDALONE) {
    mrd_servo_begin(L, MOUNT_SERVO_TYPE_L, sv); // サーボモータの通信初期設定. Serial1
    mrd_servo_begin(R, MOUNT_SERVO_TYPE_R, sv); // サーボモータの通信初期設定. Serial2
    mrd_pwm_begin();                            // PWMサーボ(本体ピン/PCA9685)の開始
  }
  mrd_disp.servo_protocol(L, MOUNT_SERVO_TYPE_L); // サーボプロトコルの表示
  mrd_disp.servo_protocol(R, MOUNT_SERVO_TYPE_R);

  // マウントされたサーボIDの表示
  mrd_disp.servo_mounts_2lines(sv);

  // SDカードの初期設定とチェック
  mrd_sd_init(MOUNT_SD, PIN_CHIPSELECT_SD);
  mrd_sd_check(MOUNT_SD, PIN_CHIPSELECT_SD, CHECK_SD_RW);

  // I2Cの初期化と開始
  bool imuahrs_mounted = mrd_wire0_setup(ImuAhrsType(MOUNT_IMUAHRS), I2C0_SPEED, ahrs, PIN_I2C0_SDA, PIN_I2C0_SCL);

  // I2Cスレッドの開始
  if (MOUNT_IMUAHRS == BNO055_AHRS) {
    xTaskCreatePinnedToCore(mrd_wire0_Core0_bno055_r, "Core0_bno055_r", 4096, NULL, 2, &thp[0], 0);
    Serial.println("Core0 thread for BNO055 start.");
    delay(10);
  } else if ((MOUNT_IMUAHRS == MPU6050_IMU || MOUNT_IMUAHRS == MPU9250_IMU) && imuahrs_mounted) {
    xTaskCreatePinnedToCore(mrd_wire0_Core0_mpu_r, "Core0_mpu_r", 4096, NULL, 2, &thp[0], 0);
    Serial.println("Core0 thread for MPU6050/MPU9250 start.");
    delay(10);
  }

  // WiFiの初期化と開始
  if (!MODE_ETHER) { // MODE_ETHER = 0 ならWiFiの初期化
    mrd_disp.esp_wifi(WIFI_AP_SSID);
    if (MODE_FIXED_IP) { // 固定IPを使用する場合はwifi.configの設定を使用する
      IPAddress fixed_ip = mrd_parse_ip_address(FIXED_IP_ADDR, Serial);
      IPAddress fixed_gw = mrd_parse_ip_address(FIXED_IP_GATEWAY, Serial);
      IPAddress fixed_sb = mrd_parse_ip_address(FIXED_IP_SUBNET, Serial);
      if (mrd_validate_network_config(fixed_ip, fixed_gw, fixed_sb, Serial)) { // IPチェック
        WiFi.config(fixed_ip, fixed_gw, fixed_sb);                             // 固定IPを設定
        Serial.println("FIXEDIP****");
      } else { // IPのパースが失敗なら停止
        mrd_error_stop(mrd_pin_err_led(), "Please Check '#define FIXED_IP_ADDR, FIXED_IP_GATEWAY, FIXED_IP_SUBNET' in 'keys.h'", Serial);
      }
    }
    if (mrd_wifi_init(udp, WIFI_AP_SSID, WIFI_AP_PASS, Serial)) {  // wifiの初期化
      mrd_disp.esp_ip(MODE_FIXED_IP, WIFI_SEND_IP, FIXED_IP_ADDR); // wifiIPの表示
    }

  } else { // MODE_ETHER = 1 ならEthernet初期化

    byte ether_mac[6];
    if (parseMacAddress(ETHER_MAC, ether_mac)) {

      if (mrd_ether_init(udp_et, PIN_CHIPSELECT_LAN, ether_mac, Serial)) {
        // Ethernet送信先IPの事前パース
        ether_send_ip = mrd_parse_ip_address(ETHER_GATEWAY, Serial);

        if (ether_send_ip == IPAddress(0, 0, 0, 0)) {
          // エラー状態でシステム停止（LEDで視覚的に通知）
          mrd_error_stop(mrd_pin_err_led(), "ERROR: Ethernet initialization failed. Fix WIFI_SEND_IP and restart", Serial);
        }
      } else {
        mrd_error_stop(mrd_pin_err_led(), "ERROR: Ethernet initialization failed. Check Ethernet hardware/config.", Serial);
      }
    } else {
      Serial.print("ERROR: Failed to parse MAC address ");
      Serial.println(ETHER_MAC);
      mrd_error_stop(mrd_pin_err_led(), "Please check '#define ETHER_MAC' in 'keys.h'", Serial);
    }
  }

  // コントロールパッドの種類を表示
  mrd_disp.mounted_pad(MOUNT_PAD);

  // Bluetoothの開始と表示(WIIMOTE)
  if (MOUNT_PAD == WIIMOTE) { // Bluetooth用スレッドの開始
    mrd_bt_settings(MOUNT_PAD, PAD_INIT_TIMEOUT, wiimote, mrd_pin_led_bt(), Serial);
    xTaskCreatePinnedToCore(Core0_BT_r, "Core0_BT_r", 2048, NULL, 5, &thp[2], 0);
  }

  // UDP開始用ダミーデータの生成
  s_udp_meridim.sval[MRD_MASTER] = 90;
  s_udp_meridim.sval[MRD_CKSM] = mrd.cksm_val(s_udp_meridim.sval, MRDM_LEN);
  r_udp_meridim.sval[MRD_MASTER] = 90;
  r_udp_meridim.sval[MRD_CKSM] = mrd.cksm_val(r_udp_meridim.sval, MRDM_LEN);
  memcpy(r_udp_meridim_last.bval, r_udp_meridim.bval, MRDM_LEN * 2);

  // タイマーの設定
  timer_semaphore = xSemaphoreCreateBinary(); // セマフォの作成
  timer = timerBegin(0, 80, true);            // タイマーの設定(1つ目のタイマーを使用, 分周比80)

  timerAttachInterrupt(timer, &frame_timer, true);     // frame_timer関数をタイマーの割り込みに登録
  timerAlarmWrite(timer, FRAME_DURATION * 1000, true); // タイマーを10msごとにトリガー
  timerAlarmEnable(timer);                             // タイマーを開始

  // 開始メッセージ
  mrd_disp.flow_start_lite_esp();
  if (MODE_ESP32_STANDALONE) {
    Serial.println("CAUTION: This is ESP32 Standalone Mode. Servo communication will be ignored.");
  }

  // タイマーの初期化
  count_frame = 0;
  portENTER_CRITICAL(&timer_mux);
  count_timer = 0;
  portEXIT_CRITICAL(&timer_mux);
}

//==================================================================================================
// MAIN LOOP
//==================================================================================================
void loop() {

  //------------------------------------------------------------------------------------
  //  [ 1 ] UDP送信
  //------------------------------------------------------------------------------------
  mrd.monitor_check_flow("[1]", monitor.flow); // デバグ用フロー表示

  // @[1-1] UDP送信の実行
  if (flg.udp_send_mode) // UDPの送信実施フラグの確認(モード確認)
  {
    flg.udp_busy = true; // UDP使用中フラグをアゲる
    if (!MODE_ETHER) {   // 0ならwifi通信
      mrd_wifi_udp_send(s_udp_meridim.bval, MRDM_BYTE, udp);
    } else { // 1なら有線LAN通信
      // 事前にパース済みのIPアドレスを使用
      mrd_ether_udp_send(s_udp_meridim.bval, MRDM_BYTE, udp_et, ether_send_ip, UDP_SEND_PORT);
    }
    flg.udp_busy = false; // UDP使用中フラグをサゲる
    flg.udp_rcvd = false; // UDP受信完了フラグをサゲる
  }

  //------------------------------------------------------------------------------------
  //  [ 2 ] UDP受信
  //------------------------------------------------------------------------------------
  mrd.monitor_check_flow("[2]", monitor.flow); // デバグ用フロー表示

  // @[2-1] UDPの受信待ち受けループ
  if (flg.udp_receive_mode) // UDPの受信実施フラグの確認(モード確認)
  {
    unsigned long start_tmp = millis();
    flg.udp_busy = true;  // UDP使用中フラグをアゲる
    flg.udp_rcvd = false; // UDP受信完了フラグをサゲる
    while (!flg.udp_rcvd) {
      // UDP受信処理
      if (!MODE_ETHER) {                                              // 0ならwifi通信
        if (mrd_wifi_udp_receive(r_udp_meridim.bval, MRDM_BYTE, udp)) // 受信確認
        {
          flg.udp_rcvd = true; // UDP受信完了フラグをアゲる
        }
      } else {                                                            // 1なら有線LAN通信
        if (mrd_ether_udp_receive(r_udp_meridim.bval, MRDM_BYTE, udp_et)) // 受信確認
        {
          flg.udp_rcvd = true; // UDP受信完了フラグをアゲる
        }
      }
      // タイムアウト抜け処理
      unsigned long current_tmp = millis();
      if (current_tmp - start_tmp >= UDP_TIMEOUT) {
        if (millis() > MONITOR_SUPPRESS_DURATION) { // 起動直後はエラー表示を抑制
          Serial.println("UDP timeout");
        }
        flg.udp_rcvd = false;
        break;
      }
      delay(1);
    }
  }
  flg.udp_busy = false; // UDP使用中フラグをサゲる

  // @[2-2] チェックサムを確認
  if (mrd.cksm_rslt(r_udp_meridim.sval, MRDM_LEN)) // Check sum OK!
  {
    mrd.monitor_check_flow("CsOK", monitor.flow); // デバグ用フロー表示

    // @[2-3] UDP受信配列から UDP送信配列にデータを転写
    memcpy(s_udp_meridim.bval, r_udp_meridim.bval, MRDM_LEN * 2);
    // EEPROMのデータを運ぶフレーム(10101)はサーボの指令ではないので, 最後のサーボの指令として残さない
    if (r_udp_meridim.sval[MRD_MASTER] != MCMD_EEPROM_SAVE_TRIM) {
      memcpy(r_udp_meridim_last.bval, r_udp_meridim.bval, MRDM_LEN * 2);
    }

    // @[2-4a] エラービット14番(ESP32のPCからのUDP受信エラー検出)をサゲる
    mrd_clear_bit16(s_udp_meridim.usval[MRD_ERR], ERRBIT_14_PC_ESP);

    if (s_udp_meridim.sval[0] == MCMD_EEPROM_SAVE_TRIM) {
      Serial.println(r_udp_meridim.sval[0]);
    }

  } else // チェックサムがNGなら最後に正しく受信したデータを使用する
  {
    // s_udp_meridim は前回の返信(サーボの現在位置やEEPROMの内容)なので, そのまま指令として使わない
    memcpy(s_udp_meridim.bval, r_udp_meridim_last.bval, MRDM_LEN * 2);

    // @[2-4b] エラービット14番(ESP32のPCからのUDP受信エラー検出)をアゲる
    mrd_set_bit16(s_udp_meridim.usval[MRD_ERR], ERRBIT_14_PC_ESP);
    err.pc_esp++;
    mrd.monitor_check_flow("CsErr*", monitor.flow); // デバグ用フロー表示
  }

  // @[2-5] シーケンス番号チェック
  mrdsq.r_expect = mrd_seq_predict_num(mrdsq.r_expect); // シーケンス番号予想値の生成

  // @[2-6] シーケンス番号のシリアルモニタ表示
  mrd_disp.seq_number(mrdsq.r_expect, r_udp_meridim.usval[MRD_SEQ], monitor.seq_num);

  if (mrd.seq_compare_nums(mrdsq.r_expect, int(s_udp_meridim.usval[MRD_SEQ]))) {

    // エラービット10番[ESP受信のスキップ検出]をサゲる
    mrd_clear_bit16(s_udp_meridim.usval[MRD_ERR], ERRBIT_10_UDP_ESP_SKIP);
    flg.meridim_rcvd = true; // Meridim受信成功フラグをアゲる.

  } else {                                              // 受信シーケンス番号の値が予想と違ったら
    mrdsq.r_expect = int(s_udp_meridim.usval[MRD_SEQ]); // 現在の受信値を予想結果としてキープ

    // エラービット10番[ESP受信のスキップ検出]をアゲる
    mrd_set_bit16(s_udp_meridim.usval[MRD_ERR], ERRBIT_10_UDP_ESP_SKIP);

    err.esp_skip++;
    flg.meridim_rcvd = false; // Meridim受信成功フラグをサゲる.
  }

  //------------------------------------------------------------------------------------
  //  [ 3 ] MasterCommand group1 の処理
  //------------------------------------------------------------------------------------
  mrd.monitor_check_flow("[3]", monitor.flow); // デバグ用フロー表示

  // @[3-1] MasterCommand group1 の処理
  // 10101 のフレームでは [20]〜[79] がEEPROMのデータ(設定ワードとトリム)のため, コマンドの実行後に
  // サーボのスロットを最後のサーボの指令に戻す. 戻さないと設定ワードがサーボのコマンドとして, トリムが目標値として処理される.
  // 書けたときはトリムが RAM に反映され原点(位置0)が新しいトリムの位置になるため, 目標値を0にする
  // (コマンドは直前のまま. トルクオンだったスロットは同じフレームの [8] で原点でトルクオンに戻る)
  const bool trim_frame_tmp = (s_udp_meridim.sval[MRD_MASTER] == MCMD_EEPROM_SAVE_TRIM);
  const bool done_tmp = execute_master_command_1(s_udp_meridim, flg.meridim_rcvd, sv, Serial);
  if (trim_frame_tmp) {
    if (done_tmp) {
      for (int i = 0; i < MRD_SERVO_SLOTS; i++) {
        r_udp_meridim_last.sval[MRD_L_ORIGIDX + 1 + i * 2] = 0;
        r_udp_meridim_last.sval[MRD_R_ORIGIDX + 1 + i * 2] = 0;
      }
    }
    memcpy(&s_udp_meridim.sval[MRD_L_ORIGIDX], &r_udp_meridim_last.sval[MRD_L_ORIGIDX],
           sizeof(short) * (MRD_R_ORIGIDX + MRD_SERVO_SLOTS * 2 - MRD_L_ORIGIDX));
  }

  //------------------------------------------------------------------------------------
  //  [ 4 ] センサー類読み取り
  //------------------------------------------------------------------------------------
  mrd.monitor_check_flow("[4]", monitor.flow); // デバグ用フロー表示

  // @[4-1] センサ値のMeridimへの転記
  meriput90_ahrs(s_udp_meridim, ahrs.read, MOUNT_IMUAHRS); // BNO055_AHRS

  //------------------------------------------------------------------------------------
  //  [ 5 ] リモコンの読み取り
  //------------------------------------------------------------------------------------
  mrd.monitor_check_flow("[5]", monitor.flow); // デバグ用フロー表示

  // @[5-1] リモコンデータの書き込み
  if (MOUNT_PAD > 0) { // リモコンがマウントされていれば

    // リモコンデータの読み込み
    pad_array.ui64val = mrd_pad_read(MOUNT_PAD, pad_array.ui64val);

    // リモコンの値をmeridimに格納する
    meriput90_pad(s_udp_meridim, pad_array, PAD_BUTTON_MARGE);
  }

  //------------------------------------------------------------------------------------
  //  [ 6 ] MasterCommand group2 の処理
  //------------------------------------------------------------------------------------
  mrd.monitor_check_flow("[6]", monitor.flow); // デバグ用フロー表示

  // @[6-1] MasterCommand group2 の処理
  execute_master_command_2(s_udp_meridim, flg.meridim_rcvd, sv, Serial);

  //------------------------------------------------------------------------------------
  //  [ 7 ] ESP32内部で位置制御する場合の処理
  //------------------------------------------------------------------------------------
  mrd.monitor_check_flow("[7]", monitor.flow); // デバグ用フロー表示

  // @[7-1] 前回読み込んだサーボ位置を保持し, 受信した目標値をサーボ配列に書き込む.
  //        1フレームあたりの指令値の変化量はSERVO_STEP_MAX[degree]に制限する.
  //        制限の基準は前回の指令値. 脱力中(コマンドが1以外)は読み込んだ実角度を基準にし,
  //        トルクオン時に現在の姿勢から動き出すようにする.
  //        起動直後で基準がないスロットは, 0度を基準にせず受信した値をそのまま使う(初回のみ).
  for (int line = 0; line < 2; line++) { // 0:L系統, 1:R系統
    float *tgt_tmp = (line == 0) ? sv.ixl_tgt : sv.ixr_tgt;
    float *tgt_past_tmp = (line == 0) ? sv.ixl_tgt_past : sv.ixr_tgt_past;
    float *cmd_past_tmp = (line == 0) ? sv.ixl_cmd_past : sv.ixr_cmd_past;
    bool *base_ok_tmp = (line == 0) ? sv.ixl_base_ok : sv.ixr_base_ok;
    const int orig_tmp = (line == 0) ? MRD_L_ORIGIDX : MRD_R_ORIGIDX;
    for (int i = 0; i < sv.num_max; i++) {
      const short cmd_tmp = s_udp_meridim.sval[orig_tmp + i * 2];
      const float val_tmp = s_udp_meridim.sval[orig_tmp + 1 + i * 2] * 0.01f; // 受信したdegree
      tgt_past_tmp[i] = tgt_tmp[i];                                          // 前回のdegreeをキープ
      if (!base_ok_tmp[i]) { // 起動直後: 基準がないので制限しない
        tgt_tmp[i] = val_tmp;
      } else if (mrd_servo_cmd(cmd_tmp) != 1) { // 脱力中: 読み込んだ実角度を基準にする
        tgt_tmp[i] = mrd_step_limit(val_tmp, tgt_past_tmp[i], SERVO_STEP_MAX);
      } else { // トルクON: 前回の指令値から SERVO_STEP_MAX 以内で目標に近づける
        tgt_tmp[i] = mrd_step_limit(val_tmp, cmd_past_tmp[i], SERVO_STEP_MAX);
      }
      cmd_past_tmp[i] = tgt_tmp[i]; // 今回の指令値を次回の基準としてキープ
      base_ok_tmp[i] = true;
    }
  }

  // @[7-2] ESP32による次回動作の計算
  // 以下はリモコンの左十字キー左右でL系統0番サーボ(首部)を30度左右にふるサンプル
  if (s_udp_meridim.sval[MRD_PAD_BUTTONS] == PAD_RIGHT) {
    sv.ixl_tgt[0] = -30.00; // -30度
  } else if (s_udp_meridim.sval[MRD_PAD_BUTTONS] == PAD_LEFT) {
    sv.ixl_tgt[0] = 30.00; // +30度
  }

  // @[7-3] 各種処理

  //------------------------------------------------------------------------------------
  //  [ 8 ] サーボ動作の実行
  //------------------------------------------------------------------------------------
  mrd.monitor_check_flow("[8]", monitor.flow); // デバグ用フロー表示

  // @[8-1] サーボ受信値の処理
  if (!MODE_ESP32_STANDALONE) {                                                      // サーボ処理を行うかどうか
    mrd_servo_drive_lite(s_udp_meridim, MOUNT_SERVO_TYPE_L, MOUNT_SERVO_TYPE_R, sv); // サーボ動作を実行する
  } else {
    // ボード単体動作モードの場合はサーボ処理をせずL0番サーボ値として+-30度のサインカーブ値を返す
    tmr.count_loop += tmr.count_loop_dlt; // サイン計算用の循環カウンタを進める
    if (tmr.count_loop > tmr.count_loop_max) {
      tmr.count_loop = 0;
    }
    sv.ixl_tgt[0] = sin(tmr.count_loop * M_PI / 180.0) * 30;
  }

  //------------------------------------------------------------------------------------
  //  [ 9 ] サーボ受信値の処理
  //------------------------------------------------------------------------------------
  mrd.monitor_check_flow("[9]", monitor.flow); // デバグ用フロー表示

  // @[9-1] サーボIDごとにの現在位置もしくは計算結果を配列に格納
  //        PWM専用スロット(種類1/11)と戻り値のないサーボ(PWM, XBUS, 動作中のHitec)は指令値を返す.
  //        マウントしていないスロットは, SERVO_UNMOUNT_REPLY が0ならコマンド・角度とも0を返し, 1なら受信した値をそのまま返す.
  //        ボード単体動作モードでは, L系統0番はサーボの設定がなくてもマウントしているとみなし, [8-1] のサインカーブを返す.
  for (int i = 0; i < MRD_SERVO_SLOTS; i++) {
    const bool standalone_sine_tmp = (MODE_ESP32_STANDALONE && i == 0);
    mrd_servo_slot_reply(s_udp_meridim, MRD_L_ORIGIDX, i, IXL_MT[i],
                         standalone_sine_tmp || (MOUNT_SERVO_TYPE_L != NOSERVO && i < sv.num_max && sv.ixl_mount[i]),
                         &sv.ixl_tgt[i]);
    mrd_servo_slot_reply(s_udp_meridim, MRD_R_ORIGIDX, i, IXR_MT[i],
                         MOUNT_SERVO_TYPE_R != NOSERVO && i < sv.num_max && sv.ixr_mount[i], &sv.ixr_tgt[i]);
  }

  //------------------------------------------------------------------------------------
  //  [ 10 ] エラーリポートの作成
  //------------------------------------------------------------------------------------
  mrd.monitor_check_flow("[10]", monitor.flow); // デバグ用フロー表示

  // @[10-1] エラーリポートの表示
  // mrd_msg_all_err(err, monitor.all_err);
  mrd_disp.all_err(MONITOR_ERR_ALL, err);

  //------------------------------------------------------------------------------------
  //  [ 11 ] MasterCommand group3 の処理
  //------------------------------------------------------------------------------------
  mrd.monitor_check_flow("[11]", monitor.flow); // デバグ用フロー表示

  execute_master_command_3(s_udp_meridim, flg.meridim_rcvd, sv, Serial);

  //------------------------------------------------------------------------------------
  //  [ 12 ] UDP送信信号作成
  //------------------------------------------------------------------------------------
  mrd.monitor_check_flow("[12]", monitor.flow); // デバグ用フロー表示

  // @[12-1] フレームスキップ検出用のカウントをカウントアップして送信用に格納
  mrdsq.s_increment = mrd.seq_increase_num(mrdsq.s_increment);
  s_udp_meridim.usval[1] = mrdsq.s_increment;

  // @[12-2] エラーが出たサーボのインデックス番号を格納
  s_udp_meridim.ubval[MRD_ERR_l] = mrd_servo_errcode_make_lite(sv);

  // @[12-3] チェックサムを計算して格納
  mrd_meriput90_cksm(s_udp_meridim);

  //------------------------------------------------------------------------------------
  //   [ 13 ] フレーム終端処理
  //------------------------------------------------------------------------------------
  mrd.monitor_check_flow("[13]", monitor.flow); // 動作チェック用シリアル表示

  // @[13-1] count_timerがcount_frameに追いつくまで待機
  count_frame++;
  while (true) {
    if (xSemaphoreTake(timer_semaphore, 0) == pdTRUE) {
      portENTER_CRITICAL(&timer_mux);
      unsigned long current_count_timer = count_timer; // ハードウェアタイマーの値を読む
      portEXIT_CRITICAL(&timer_mux);
      if (current_count_timer >= count_frame) {
        break;
      }
    }
  }

  // @[13-2] 必要に応じてフレームの遅延累積時間frameDelayをリセット
  if (flg.count_frame_reset) {
    portENTER_CRITICAL(&timer_mux);
    count_frame = count_timer;
    portEXIT_CRITICAL(&timer_mux);
    flg.count_frame_reset = false; // リセットは1回だけ(残すと毎フレームリセットされ遅れの追い付きが効かない)
  }

  mrd.monitor_check_flow("\n", monitor.flow); // 動作チェック用シリアル表示
}

#endif // __MERIDIAN_LITE_MAIN__
