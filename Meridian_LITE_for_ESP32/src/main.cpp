#define VERSION "Meridian_LITE_v1.1.1_2026.04.28" // バージョン表示

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
#define CONFIG_DEFINE_ARRAYS // サーボ設定配列の実体をここで定義
#include "main.h"
#include "config.h"
#include "keys.h"

// 常時必要なモジュール
#include "mrd_command.h"
#include "mrd_disp.h"
#include "mrd_eeprom.h"
#include "mrd_move.h"
#include "mrd_servo.h"
#include "mrd_util.h"
#include "mrd_wire0.h" // AhrsValue定義のため常時必要

// 通信モジュール（排他選択）
#if MODE_ETHER
#include "mrd_ether.h"
#else
#include "mrd_wifi.h"
#endif

// オプションモジュール: SDカード
#if MOUNT_SD
#include "mrd_sd.h"
#endif

// オプションモジュール: Bluetoothパッド
#if MOUNT_PAD == WIIMOTE || MOUNT_PAD == WIIMOTE_C
#include "mrd_bt_pad.h"
#endif

MERIDIANFLOW::Meridian mrd;
IcsHardSerialClass ics_L(&Serial1, PIN_EN_L, SERVO_BAUDRATE_L, SERVO_TIMEOUT_L);
IcsHardSerialClass ics_R(&Serial2, PIN_EN_R, SERVO_BAUDRATE_R, SERVO_TIMEOUT_R);

// ライブラリ導入
#include <Arduino.h>

//==================================================================================================
//  グローバル変数の実体定義
//==================================================================================================

// main.hで宣言された変数
TaskHandle_t thp[4] = {NULL};
Meridim90Union s_udp_meridim = {0}; // ゼロ初期化
Meridim90Union r_udp_meridim = {0}; // ゼロ初期化
MrdFlags flg;
MrdSq mrdsq;
MrdTimer tmr;
MrdErr err;
PadUnion pad_array = {0};
PadValue pad_analog;
AhrsValue ahrs;
ServoParam sv;
MrdMonitor monitor;
MrdMsgHandler mrd_disp(Serial);
SemaphoreHandle_t ahrs_mutex; // AHRSデータアクセス用mutex
SemaphoreHandle_t pad_mutex;  // PADデータアクセス用mutex

// mrd_wifi.hで宣言された変数
#if !MODE_ETHER
WiFiUDP udp;
#endif

// mrd_bt_pad.hで宣言された変数
ESP32Wiimote wiimote;

// (eeprom_write_data, eeprom_read_data は未使用のため削除済み)

// ハードウェアタイマーとカウンタ用変数の定義
hw_timer_t *timer = NULL;                              // ハードウェアタイマーの設定
SemaphoreHandle_t timer_semaphore;                     // ハードウェアタイマー用のセマフォ
portMUX_TYPE timer_mux = portMUX_INITIALIZER_UNLOCKED; // ハードウェアタイマー用のミューテックス
unsigned long count_frame = 0;                         // フレーム処理の完了時にカウントアップ
volatile unsigned long count_timer = 0;                // フレーム用タイマーのカウントアップ

// Ethernet送信先IP(事前パース)
IPAddress ether_send_ip(0, 0, 0, 0); // Ethernet送信先IP(初期化)

/// @brief count_timerを保護しつつ1ずつインクリメント.
void IRAM_ATTR frame_timer() {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  portENTER_CRITICAL_ISR(&timer_mux);
  count_timer++;
  portEXIT_CRITICAL_ISR(&timer_mux);
  xSemaphoreGiveFromISR(timer_semaphore, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken); // 高優先度タスクへの即時切替
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
  unsigned long start_time = millis();                             // シリアルモニターの確立待ち
  while (!Serial && (millis() - start_time < SERIAL_PC_TIMEOUT)) { // タイムアウトもチェック
    delay(1);
  }

  // ピンモードの設定
  pinMode(PIN_ERR_LED, OUTPUT); // エラー通知用LED

  // ボード搭載コンデンサの充電時間として待機
  mrd_disp.charging(CHARGE_TIME);

  // 起動メッセージの表示(バージョン, PC-USB, SPI0, I2C0のスピード)
  mrd_disp.hello_lite_esp(VERSION, SERIAL_PC_BPS, SPI0_SPEED, I2C0_SPEED);

  // サーボ値の初期設定
  sv.num_max = max(mrd_max_used_index(IXL_MT, IXL_MAX),
                   mrd_max_used_index(IXR_MT, IXR_MAX)); // サーボ処理回数
  for (int i = 0; i < sv.num_max; i++) {                 // configで設定した値を反映
    sv.ixl_mount[i] = IXL_MT[i];
    sv.ixr_mount[i] = IXR_MT[i];
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
  mrd_disp.servo_bps_2lines(SERVO_BAUDRATE_L, SERVO_BAUDRATE_R);

  // サーボ用UART設定
  mrd_servo_begin(L, MOUNT_SERVO_TYPE_L);         // サーボモータの通信初期設定(Serial1)
  mrd_servo_begin(R, MOUNT_SERVO_TYPE_R);         // サーボモータの通信初期設定(Serial2)
  mrd_disp.servo_protocol(L, MOUNT_SERVO_TYPE_L); // L系統プロトコルの表示
  mrd_disp.servo_protocol(R, MOUNT_SERVO_TYPE_R); // R系統プロトコルの表示

  // マウントされたサーボIDの表示
  mrd_disp.servo_mounts_2lines(sv);

  // EEPROMの開始
  Serial.print("Initializing EEPROM... ");
  if (mrd_eeprom_init(EEPROM_SIZE)) {
    Serial.println("OK");
  } else {
    Serial.println("Failed");
  }

  // EEPROMにconfigのサーボ設定値を書き込む場合
  if (EEPROM_SET) {
    Serial.println("Set EEPROM data from config.");
    // 書込データの作成と書込
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
  mrd_eeprom_dump_at_boot(EEPROM_DUMP, EEPROM_STYLE, Serial);

  // EEPROMのリードライトテスト
  // mrd_eeprom_write_read_check(mrd_eeprom_make_data_from_config(),
  //                             CHECK_EEPROM_RW, EEPROM_PROTECT, EEPROM_STYLE);

  // SDカードの初期設定とチェック
#if MOUNT_SD
  mrd_sd_init(MOUNT_SD, PIN_CHIPSELECT_SD, Serial);
  mrd_sd_check(MOUNT_SD, PIN_CHIPSELECT_SD, CHECK_SD_RW, Serial);
#endif

  // I2Cの初期化と開始
  mrd_wire0_setup(MOUNT_IMUAHRS, I2C0_SPEED, ahrs, PIN_I2C0_SDA, PIN_I2C0_SCL);

  // AHRSデータ用mutexの作成
  ahrs_mutex = xSemaphoreCreateMutex();
  if (ahrs_mutex == NULL) {
    mrd_error_stop(PIN_ERR_LED, "ERROR: Failed to create AHRS mutex", Serial);
  }

  // PADデータ用mutexの作成
  pad_mutex = xSemaphoreCreateMutex();
  if (pad_mutex == NULL) {
    mrd_error_stop(PIN_ERR_LED, "ERROR: Failed to create PAD mutex", Serial);
  }

  // I2C用スレッドの開始
  if (MOUNT_IMUAHRS == BNO055_AHRS) {
    BaseType_t result = xTaskCreatePinnedToCore(mrd_wire0_Core0_bno055_r, "Core0_bno055_r", 4096, NULL, 2, &thp[0], 0);
    if (result != pdPASS) {
      mrd_error_stop(PIN_ERR_LED, "ERROR: Failed to create BNO055 task", Serial);
    }
    Serial.println("Core0 thread for BNO055 start.");
    delay(10);
  }

  // 通信モジュールの初期化
#if MODE_ETHER
  // Ethernet初期化
  byte ether_mac[6];
  if (parseMacAddress(ETHER_MAC, ether_mac)) {
    if (mrd_ether_init(udp_et, PIN_CHIPSELECT_LAN, ether_mac, Serial)) {
      // Ethernet送信先IPを事前パース
      ether_send_ip = mrd_parse_ip_address(ETHER_GATEWAY, Serial);
      if (ether_send_ip == IPAddress(0, 0, 0, 0)) {
        mrd_error_stop(PIN_ERR_LED, "ERROR: Ethernet initialization failed. Fix ETHER_GATEWAY and restart", Serial);
      }
    } else {
      mrd_error_stop(PIN_ERR_LED, "ERROR: Ethernet initialization failed. Check Ethernet hardware/config.", Serial);
    }
  } else {
    Serial.print("ERROR: Failed to parse MAC address ");
    Serial.println(ETHER_MAC);
    mrd_error_stop(PIN_ERR_LED, "Please check '#define ETHER_MAC' in 'keys.h'", Serial);
  }
#else
  // WiFi初期化
  mrd_disp.esp_wifi(WIFI_AP_SSID);
  if (MODE_FIXED_IP) { // 固定IP使用時はwifi.configの設定を使用
    IPAddress fixed_ip = mrd_parse_ip_address(FIXED_IP_ADDR, Serial);
    IPAddress fixed_gw = mrd_parse_ip_address(FIXED_IP_GATEWAY, Serial);
    IPAddress fixed_sb = mrd_parse_ip_address(FIXED_IP_SUBNET, Serial);
    if (mrd_validate_network_config(fixed_ip, fixed_gw, fixed_sb, Serial)) { // IP検証
      WiFi.config(fixed_ip, fixed_gw, fixed_sb);                             // 固定IPを設定
      Serial.println("FIXEDIP****");
    } else { // IPパース失敗時は停止
      mrd_error_stop(PIN_ERR_LED, "Please Check '#define FIXED_IP_ADDR, FIXED_IP_GATEWAY, FIXED_IP_SUBNET' in 'keys.h'", Serial);
    }
  }
  if (mrd_wifi_init(udp, WIFI_AP_SSID, WIFI_AP_PASS, Serial)) {  // WiFi初期化
    mrd_disp.esp_ip(MODE_FIXED_IP, WIFI_SEND_IP, FIXED_IP_ADDR); // WiFi IPの表示
  } else {
    mrd_error_stop(PIN_ERR_LED, "ERROR: WiFi initialization failed. Check SSID/password in 'keys.h'", Serial);
  }
#endif

  // コントロールパッドの種類を表示
  mrd_disp.mounted_pad(MOUNT_PAD);

  // Bluetoothの開始(WIIMOTE)
#if MOUNT_PAD == WIIMOTE || MOUNT_PAD == WIIMOTE_C
  if (mrd_bt_settings(MOUNT_PAD, PAD_INIT_TIMEOUT, wiimote, PIN_LED_BT, Serial)) {
    BaseType_t result = xTaskCreatePinnedToCore(Core0_BT_r, "Core0_BT_r", 2048, NULL, 5, &thp[2], 0);
    if (result != pdPASS) {
      mrd_error_stop(PIN_ERR_LED, "ERROR: Failed to create Wiimote task", Serial);
    }
    Serial.println("Core0 thread for Wiimote start.");
  }
  // mrd_bt_settingsが失敗した場合はタイムアウトメッセージが表示済み
#endif

  // UDP開始用ダミーデータの生成
  s_udp_meridim.sval[MRD_MASTER] = 90;
  s_udp_meridim.sval[MRD_CKSM] = mrd.cksm_val(s_udp_meridim.sval, MRDM_LEN);
  r_udp_meridim.sval[MRD_MASTER] = 90;
  r_udp_meridim.sval[MRD_CKSM] = mrd.cksm_val(r_udp_meridim.sval, MRDM_LEN);

  // タイマーの設定
  timer_semaphore = xSemaphoreCreateBinary(); // セマフォ作成
  if (timer_semaphore == NULL) {
    mrd_error_stop(PIN_ERR_LED, "ERROR: Failed to create timer semaphore", Serial);
  }
  timer = timerBegin(0, 80, true); // タイマー設定(1つ目のタイマー, 分周比80)

  timerAttachInterrupt(timer, &frame_timer, true);     // frame_timer関数を割込みに登録
  timerAlarmWrite(timer, FRAME_DURATION * 1000, true); // タイマーを10ms毎にトリガー
  timerAlarmEnable(timer);                             // タイマー開始

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
  if (flg.udp_send_mode) { // UDP送信実施フラグの確認(モード確認)
    // [DEBUG] UDP送信直前のsval[21]を表示
    Serial.print("[DBG_TX] sval[21]=");
    Serial.println(s_udp_meridim.sval[21]);

    flg.udp_busy = true;   // UDP使用中フラグをセット
#if MODE_ETHER
    // 有線LAN通信: 事前パース済みのIPアドレスを使用
    mrd_ether_udp_send(s_udp_meridim.bval, MRDM_BYTE, udp_et, ether_send_ip, UDP_SEND_PORT);
#else
    // WiFi通信
    mrd_wifi_udp_send(s_udp_meridim.bval, MRDM_BYTE, udp);
#endif
    flg.udp_busy = false; // UDP使用中フラグをクリア
    flg.udp_rcvd = false; // UDP受信完了フラグをクリア
  }

  //------------------------------------------------------------------------------------
  //  [ 2 ] UDP受信
  //------------------------------------------------------------------------------------
  mrd.monitor_check_flow("[2]", monitor.flow); // デバグ用フロー表示

  // @[2-1] UDP受信待受けループ
  if (flg.udp_receive_mode) { // UDP受信実施フラグの確認(モード確認)
    unsigned long start_tmp = millis();
    flg.udp_busy = true;  // UDP使用中フラグをセット
    flg.udp_rcvd = false; // UDP受信完了フラグをクリア
    while (!flg.udp_rcvd) {
      // UDP受信処理
#if MODE_ETHER
      // 有線LAN通信
      if (mrd_ether_udp_receive(r_udp_meridim.bval, MRDM_BYTE, udp_et)) {
        flg.udp_rcvd = true; // UDP受信完了フラグをセット
      }
#else
      // WiFi通信
      if (mrd_wifi_udp_receive(r_udp_meridim.bval, MRDM_BYTE, udp)) {
        flg.udp_rcvd = true; // UDP受信完了フラグをセット
      }
#endif
      // タイムアウト処理
      unsigned long current_tmp = millis();
      if (current_tmp - start_tmp >= UDP_TIMEOUT) {
        if (current_tmp > MONITOR_SUPPRESS_DURATION) { // 起動直後はエラー表示を抑制
          Serial.println("UDP timeout");
        }
        flg.udp_rcvd = false;
        break;
      }
      delay(1);
    }
  }
  flg.udp_busy = false; // UDP使用中フラグをクリア

  // @[2-2] チェックサム確認
  if (mrd.cksm_rslt(r_udp_meridim.sval, MRDM_LEN)) { // チェックサムOK
    mrd.monitor_check_flow("CsOK", monitor.flow);    // デバグ用フロー表示

    // @[2-3] UDP受信配列から UDP送信配列にデータを転写
    memcpy(s_udp_meridim.bval, r_udp_meridim.bval, MRDM_LEN * 2);

    // @[2-4a] エラービット14番(PCからのUDP受信エラー検出)をクリア
    mrd_clear_bit16(s_udp_meridim.usval[MRD_ERR], ERRBIT_14_PC_ESP);

  } else { // チェックサムNGなら前回データを使用

    // @[2-4b] エラービット14番(PCからのUDP受信エラー検出)をセット
    mrd_set_bit16(s_udp_meridim.usval[MRD_ERR], ERRBIT_14_PC_ESP);
    err.pc_esp++;
    mrd.monitor_check_flow("CsErr*", monitor.flow); // デバグ用フロー表示
  }

  // @[2-5] シーケンス番号チェック
  mrdsq.r_expect = mrd_seq_predict_num(mrdsq.r_expect); // シーケンス番号予想値を生成

  // @[2-6] シーケンス番号をシリアルモニタに表示
  mrd_disp.seq_number(mrdsq.r_expect, r_udp_meridim.usval[MRD_SEQ], monitor.seq_num);

  if (mrd.seq_compare_nums(mrdsq.r_expect, int(s_udp_meridim.usval[MRD_SEQ]))) {

    // エラービット10番(ESP受信スキップ検出)をクリア
    mrd_clear_bit16(s_udp_meridim.usval[MRD_ERR], ERRBIT_10_UDP_ESP_SKIP);
    flg.meridim_rcvd = true; // Meridim受信成功フラグをセット

  } else {                                              // 受信シーケンス番号が予想と異なる場合
    mrdsq.r_expect = int(s_udp_meridim.usval[MRD_SEQ]); // 現在の受信値を予想値として保持

    // エラービット10番(ESP受信スキップ検出)をセット
    mrd_set_bit16(s_udp_meridim.usval[MRD_ERR], ERRBIT_10_UDP_ESP_SKIP);

    err.esp_skip++;
    flg.meridim_rcvd = false; // Meridim受信成功フラグをクリア
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

  // @[4-1] センサ値をMeridimに転記
  meriput90_ahrs(s_udp_meridim, ahrs.read, MOUNT_IMUAHRS); // BNO055_AHRS

  //------------------------------------------------------------------------------------
  //  [ 5 ] リモコンの読み取り
  //------------------------------------------------------------------------------------
  mrd.monitor_check_flow("[5]", monitor.flow); // デバグ用フロー表示

  // @[5-1] リモコンデータの書込み
  if (MOUNT_PAD > 0) { // リモコンがマウントされている場合

    // リモコンデータを読込み
    if (MOUNT_PAD == WIIMOTE) {
      // WIIMOTE: Copy pad data under mutex protection
      if (xSemaphoreTake(pad_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        PadUnion pad_local = pad_array; // Copy under mutex
        xSemaphoreGive(pad_mutex);
        // リモコン値をMeridimに格納
        meriput90_pad(s_udp_meridim, pad_local, PAD_BUTTON_MARGE);
      }
    } else {
      // Other pads: No mutex needed
      pad_array.ui64val = mrd_pad_read(MOUNT_PAD, pad_array.ui64val, ics_R);
      // リモコン値をMeridimに格納
      meriput90_pad(s_udp_meridim, pad_array, PAD_BUTTON_MARGE);
    }
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

  // @[7-1] 前回のサーボ位置をサーボ配列に書込み
  constexpr float DEGREE_SCALE = 0.01f; // 度数変換係数(ループ外で定義)
  for (int i = 0; i < sv.num_max; i++) {
    int idx_l = i * 2 + 21; // インデックス計算を一度だけ実行
    int idx_r = i * 2 + 51;
    sv.ixl_tgt_past[i] = sv.ixl_tgt[i];                       // 前回のdegreeを保持
    sv.ixr_tgt_past[i] = sv.ixr_tgt[i];                       // 前回のdegreeを保持
    sv.ixl_tgt[i] = s_udp_meridim.sval[idx_l] * DEGREE_SCALE; // 受信degreeを格納
    sv.ixr_tgt[i] = s_udp_meridim.sval[idx_r] * DEGREE_SCALE; // 受信degreeを格納
  }

  // 移動差が大きい時に緩和する補正フィルタ (clamp処理で分岐を削減)
  constexpr float TGT_GAP_MAX = 3.0f; // ギャップ最大値
  for (int i = 0; i < sv.num_max; i++) {
    // L系統: 差分をギャップ範囲内にクランプ
    float diff_l = sv.ixl_tgt[i] - sv.ixl_tgt_past[i];
    if (diff_l > TGT_GAP_MAX) {
      sv.ixl_tgt[i] = sv.ixl_tgt_past[i] + TGT_GAP_MAX;
    } else if (diff_l < -TGT_GAP_MAX) {
      sv.ixl_tgt[i] = sv.ixl_tgt_past[i] - TGT_GAP_MAX;
    }
    // R系統: 差分をギャップ範囲内にクランプ
    float diff_r = sv.ixr_tgt[i] - sv.ixr_tgt_past[i];
    if (diff_r > TGT_GAP_MAX) {
      sv.ixr_tgt[i] = sv.ixr_tgt_past[i] + TGT_GAP_MAX;
    } else if (diff_r < -TGT_GAP_MAX) {
      sv.ixr_tgt[i] = sv.ixr_tgt_past[i] - TGT_GAP_MAX;
    }
  }

  // @[7-2] ESP32による次回動作の計算
  // リモコン十字キー左右でL系統0番サーボ(首部)を30度左右に振るサンプル
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
  if (!MODE_ESP32_STANDALONE) { // サーボ処理を実行
    mrd_servo_drive_lite(s_udp_meridim, MOUNT_SERVO_TYPE_L, MOUNT_SERVO_TYPE_R, sv);
  } else {
    // ボード単体動作モードではサーボ処理をせずL0番サーボ値として+-30度のサイン波を返す
    sv.ixl_tgt[0] = sinf(tmr.count_loop * (float)M_PI / 180.0f) * 30.0f;
    // サイン波用カウンタをインクリメント
    tmr.count_loop += tmr.count_loop_dlt;
    if (tmr.count_loop > tmr.count_loop_max) {
      tmr.count_loop = 0;
    }
  }

  //------------------------------------------------------------------------------------
  //  [ 9 ] サーボ受信値の処理
  //------------------------------------------------------------------------------------
  mrd.monitor_check_flow("[9]", monitor.flow); // デバグ用フロー表示

  // @[9-1] 各サーボIDの現在位置または計算結果を配列に格納
  for (int i = 0; i < sv.num_max; i++) {
    int idx_l = i * 2 + 21; // インデックス計算を一度だけ実行
    int idx_r = i * 2 + 51;
    // 最新のサーボ角度をdegree単位で格納
    s_udp_meridim.sval[idx_l] = mrd.float2HfShort(sv.ixl_tgt[i]);
    s_udp_meridim.sval[idx_r] = mrd.float2HfShort(sv.ixr_tgt[i]);
    // [DEBUG] L0の書き戻し値を表示
    if (i == 0) {
      Serial.print("[DBG_WB] L0 tgt=");
      Serial.print(sv.ixl_tgt[0]);
      Serial.print(" sval[21]=");
      Serial.println(s_udp_meridim.sval[21]);
    }
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

  // @[12-1] フレームスキップ検出用カウントをインクリメントし送信用に格納
  mrdsq.s_increment = mrd.seq_increase_num(mrdsq.s_increment);
  s_udp_meridim.usval[1] = mrdsq.s_increment;

  // @[12-2] エラー発生サーボのインデックス番号を格納
  s_udp_meridim.ubval[MRD_ERR_l] = mrd_servo_make_errcode_lite(sv);

  // @[12-3] チェックサムを計算して格納
  mrd_meriput90_cksm(s_udp_meridim);

  //------------------------------------------------------------------------------------
  //   [ 13 ] フレーム終端処理
  //------------------------------------------------------------------------------------
  mrd.monitor_check_flow("[13]", monitor.flow); // 動作チェック用シリアル表示

  // @[13-1] count_timerがcount_frameに追いつくまで待機
  count_frame++;
  const TickType_t timeout_ticks = pdMS_TO_TICKS(FRAME_DURATION * 2); // タイムアウト設定
  while (true) {
    if (xSemaphoreTake(timer_semaphore, timeout_ticks) == pdTRUE) {
      portENTER_CRITICAL(&timer_mux);
      unsigned long current_count_timer = count_timer; // ハードウェアタイマー値を読取り
      portEXIT_CRITICAL(&timer_mux);
      if (current_count_timer >= count_frame) {
        break;
      }
    } else {
      // タイムアウト発生時はフレームカウンタを同期
      portENTER_CRITICAL(&timer_mux);
      count_frame = count_timer;
      portEXIT_CRITICAL(&timer_mux);
      break;
    }
  }

  // @[13-2] 必要に応じてフレーム遅延累積時間をリセット
  if (flg.count_frame_reset) {
    portENTER_CRITICAL(&timer_mux);
    count_frame = count_timer;
    portEXIT_CRITICAL(&timer_mux);
    flg.count_frame_reset = false; // リセット完了後にフラグをクリア
  }

  mrd.monitor_check_flow("\n", monitor.flow); // 動作チェック用シリアル表示
}
