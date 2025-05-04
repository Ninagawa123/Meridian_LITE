#ifndef __MERIDIAN_LITE_MAIN__
#define __MERIDIAN_LITE_MAIN__

#define VERSION "Meridian_LITE_v1.1.1_2025_04.29" // バージョン表示

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
#include "mrd_move.h"
#include "mrd_sd.h"
#include "mrd_servo.h"
#include "mrd_util.h"
#include "mrd_wifi.h"
#include "mrd_wire0.h"

MERIDIANFLOW::Meridian mrd;
IcsHardSerialClass ics_L(&Serial1, PIN_EN_L, SERVO_BAUDRATE_L, SERVO_TIMEOUT_L);
IcsHardSerialClass ics_R(&Serial2, PIN_EN_R, SERVO_BAUDRATE_R, SERVO_TIMEOUT_R);

// ライブラリ導入
#include <Arduino.h>

// ハードウェアタイマーとカウンタ用変数の定義
hw_timer_t *timer = NULL;                              // ハードウェアタイマーの設定
volatile SemaphoreHandle_t timer_semaphore;            // ハードウェアタイマー用のセマフォ
portMUX_TYPE timer_mux = portMUX_INITIALIZER_UNLOCKED; // ハードウェアタイマー用のミューテックス
unsigned long count_frame = 0;                         // フレーム処理の完了時にカウントアップ
volatile unsigned long count_timer = 0;                // フレーム用タイマーのカウントアップ

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
  pinMode(PIN_LED_BT, OUTPUT);
  digitalWrite(PIN_LED_BT, HIGH);

  // シリアルモニターの設定
  Serial.begin(SERIAL_PC_BPS);
  // シリアルモニターの確立待ち
  unsigned long start_time = millis();
  while (!Serial && (millis() - start_time < SERIAL_PC_TIMEOUT)) { // タイムアウトもチェック
    delay(1);
  }

  // ピンモードの設定
  pinMode(PIN_ERR_LED, OUTPUT); // エラー通知用LED

  // ボード搭載のコンデンサの充電時間として待機
  mrd_disp.charging(CHARGE_TIME);

  // 起動メッセージの表示(バージョン, PC-USB,SPI0,i2c0のスピード)
  mrd_disp.hello_lite_esp(VERSION, SERIAL_PC_BPS, SPI0_SPEED, I2C0_SPEED);

  // サーボ値の初期設定
  sv.num_max = max(mrd_max_used_index(IXL_MT, IXL_MAX),  //
                   mrd_max_used_index(IXR_MT, IXR_MAX)); // サーボ処理回数
  for (int i = 0; i <= sv.num_max; i++) {                // configで設定した値を反映させる
    sv.ixl_mount[i] = IXL_MT[i];
    sv.ixr_mount[i] = IXR_MT[i];
    sv.ixl_id[i] = IXL_ID[i];
    sv.ixr_id[i] = IXR_ID[i];
    sv.ixl_cw[i] = IXL_CW[i];
    sv.ixr_cw[i] = IXR_CW[i];
    sv.ixl_trim[i] = IDL_TRIM[i];
    sv.ixr_trim[i] = IDR_TRIM[i];
  };

  // サーボUARTの通信速度の表示
  mrd_disp.servo_bps_2lines(SERVO_BAUDRATE_L, SERVO_BAUDRATE_R);

  // サーボ用UART設定
  mrd_servo_begin(L, MOUNT_SERVO_TYPE_L);         // サーボモータの通信初期設定. Serial2
  mrd_servo_begin(R, MOUNT_SERVO_TYPE_R);         // サーボモータの通信初期設定. Serial3
  mrd_disp.servo_protocol(L, MOUNT_SERVO_TYPE_R); // サーボプロトコルの表示
  mrd_disp.servo_protocol(R, MOUNT_SERVO_TYPE_R);

  // マウントされたサーボIDの表示
  mrd_disp.servo_mounts_2lines(sv);

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

  // EEPROMからサーボ設定の内容を読み込んで反映する場合
  if (EEPROM_LOAD) {
    mrd_eeprom_load_servosettings(sv, true, Serial);
  }

  // EEPROMの内容ダンプ表示をする場合
  mrd_eeprom_dump_at_boot(EEPROM_DUMP, EEPROM_STYLE, Serial); //

  // EEPROMのリードライトテスト
  // mrd_eeprom_write_read_check(mrd_eeprom_make_data_from_config(),
  //                             CHECK_EEPROM_RW, EEPROM_PROTECT, EEPROM_STYLE);

  // SDカードの初期設定とチェック
  mrd_sd_init(MOUNT_SD, PIN_CHIPSELECT_SD);
  mrd_sd_check(MOUNT_SD, PIN_CHIPSELECT_SD, CHECK_SD_RW);

  // I2Cの初期化と開始
  mrd_wire0_setup(BNO055_AHRS, I2C0_SPEED, ahrs, PIN_I2C0_SDA, PIN_I2C0_SCL);

  // I2Cスレッドの開始
  if (MOUNT_IMUAHRS == BNO055_AHRS) {
    xTaskCreatePinnedToCore(mrd_wire0_Core0_bno055_r, "Core0_bno055_r", 4096, NULL, 2, &thp[0], 0);
    Serial.println("Core0 thread for BNO055 start.");
    delay(10);
  }

  // WiFiの初期化と開始
  mrd_disp.esp_wifi(WIFI_AP_SSID);
  if (mrd_wifi_init(udp, WIFI_AP_SSID, WIFI_AP_PASS, Serial)) {
    // wifiIPの表示
    mrd_disp.esp_ip(MODE_FIXED_IP, WIFI_SEND_IP, FIXED_IP_ADDR);
  }

  // コントロールパッドの種類を表示
  mrd_disp.mounted_pad(MOUNT_PAD);

  // Bluetoothの開始と表示(WIIMOTE)
  if (MOUNT_PAD == WIIMOTE) { // Bluetooth用スレッドの開始
    mrd_bt_settings(MOUNT_PAD, PAD_INIT_TIMEOUT, wiimote, PIN_LED_BT, Serial);
    xTaskCreatePinnedToCore(Core0_BT_r, "Core0_BT_r", 2048, NULL, 5, &thp[2], 0);
  }

  // UDP開始用ダミーデータの生成
  s_udp_meridim.sval[MRD_MASTER] = 90;
  s_udp_meridim.sval[MRD_CKSM] = mrd.cksm_val(s_udp_meridim.sval, MRDM_LEN);
  r_udp_meridim.sval[MRD_MASTER] = 90;
  r_udp_meridim.sval[MRD_CKSM] = mrd.cksm_val(r_udp_meridim.sval, MRDM_LEN);

  // タイマーの設定
  timer_semaphore = xSemaphoreCreateBinary(); // セマフォの作成
  timer = timerBegin(0, 80, true);            // タイマーの設定(1つ目のタイマーを使用, 分周比80)

  timerAttachInterrupt(timer, &frame_timer, true);     // frame_timer関数をタイマーの割り込みに登録
  timerAlarmWrite(timer, FRAME_DURATION * 1000, true); // タイマーを10msごとにトリガー
  timerAlarmEnable(timer);                             // タイマーを開始

  // 開始メッセージ
  mrd_disp.flow_start_lite_esp();

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
    mrd_wifi_udp_send(s_udp_meridim.bval, MRDM_BYTE, udp);
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
      if (mrd_wifi_udp_receive(r_udp_meridim.bval, MRDM_BYTE, udp)) // 受信確認
      {
        flg.udp_rcvd = true; // UDP受信完了フラグをアゲる
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

    // @[2-4a] エラービット14番(ESP32のPCからのUDP受信エラー検出)をサゲる
    mrd_clear_bit16(s_udp_meridim.usval[MRD_ERR], ERRBIT_14_PC_ESP);

    if (s_udp_meridim.sval[0] == MCMD_EEPROM_SAVE_TRIM) {
      Serial.println(r_udp_meridim.sval[0]);
    }

  } else // チェックサムがNGならバッファから転記せず前回のデータを使用する
  {

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
  execute_master_command_1(s_udp_meridim, flg.meridim_rcvd, sv, Serial);

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

  // @[7-1] 前回のラストに読み込んだサーボ位置をサーボ配列に書き込む
  for (int i = 0; i <= sv.num_max; i++) {
    sv.ixl_tgt_past[i] = sv.ixl_tgt[i];                    // 前回のdegreeをキープ
    sv.ixr_tgt_past[i] = sv.ixr_tgt[i];                    // 前回のdegreeをキープ
    sv.ixl_tgt[i] = s_udp_meridim.sval[i * 2 + 21] * 0.01; // 受信したdegreeを格納
    sv.ixr_tgt[i] = s_udp_meridim.sval[i * 2 + 51] * 0.01; // 受信したdegreeを格納
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
  if (!MODE_ESP32_STANDALONE) {                                                       // サーボ処理を行うかどうか
    mrd_servos_drive_lite(s_udp_meridim, MOUNT_SERVO_TYPE_L, MOUNT_SERVO_TYPE_R, sv); // サーボ動作を実行する
  } else {
    // ボード単体動作モードの場合はサーボ処理をせずL0番サーボ値として+-30度のサインカーブ値を返す
    sv.ixl_tgt[0] = sin(tmr.count_loop * M_PI / 180.0) * 30;
  }

  //------------------------------------------------------------------------------------
  //  [ 9 ] サーボ受信値の処理
  //------------------------------------------------------------------------------------
  mrd.monitor_check_flow("[9]", monitor.flow); // デバグ用フロー表示

  // @[9-1] サーボIDごとにの現在位置もしくは計算結果を配列に格納
  for (int i = 0; i <= sv.num_max; i++) {
    // 最新のサーボ角度をdegreeで格納
    s_udp_meridim.sval[i * 2 + 21] = mrd.float2HfShort(sv.ixl_tgt[i]);
    s_udp_meridim.sval[i * 2 + 51] = mrd.float2HfShort(sv.ixr_tgt[i]);
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
  s_udp_meridim.ubval[MRD_ERR_l] = mrd_servos_make_errcode_lite(sv);

  // @[12-3] チェックサムを計算して格納
  // s_udp_meridim.sval[MRD_CKSM] = mrd.cksm_val(s_udp_meridim.sval, MRDM_LEN);
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
  }

  mrd.monitor_check_flow("\n", monitor.flow); // 動作チェック用シリアル表示
}

#endif // __MERIDIAN_LITE_MAIN__
