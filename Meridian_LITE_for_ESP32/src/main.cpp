/// @file    Meridian_LITE_for_ESP32/src/main.cpp
/// @brief   Meridian is a system that smartly realizes the digital twin of a robot.
/// @details Meridian_LITE for Meridan Board -LITE- with ESP32DecKitC.
/// 
/// This code is licensed under the MIT License.
/// Copyright (c) 2022 Izumi Ninagawa & Project Meridian

#define VERSION "Meridian_LITE_v1.1.1_2024_0724" // バージョン表示

//================================================================================================================
//  初期設定
//================================================================================================================

// ヘッダファイルの読み込み
#include "main.h"

#include "config.h"
#include "keys.h"
#include "mrd_eeprom.h"
#include "mrd_move.h"
#include "mrd_msg.h"
#include "mrd_pad.h"
#include "mrd_sd.h"
#include "mrd_servo.h"
#include "mrd_wifi.h"
#include "mrd_wire0.h"

// ライブラリ導入
#include <Arduino.h>

// ハードウェアタイマーとカウンタ用変数の定義
hw_timer_t *timer = NULL;                   // ハードウェアタイマーの設定
volatile SemaphoreHandle_t timer_semaphore; // ハードウェアタイマー用のセマフォ
portMUX_TYPE timer_mux = portMUX_INITIALIZER_UNLOCKED; // ハードウェアタイマー用のミューテックス
unsigned long count_frame = 0;          // フレーム処理の完了時にカウントアップ
volatile unsigned long count_timer = 0; // フレーム用タイマーのカウントアップ

/// @brief count_timerを保護しつつ1ずつインクリメント
void IRAM_ATTR frame_timer() {
  portENTER_CRITICAL_ISR(&timer_mux);
  count_timer++;
  portEXIT_CRITICAL_ISR(&timer_mux);
  xSemaphoreGiveFromISR(timer_semaphore, NULL); // セマフォを与える
}

//================================================================================================================
//  SETUP
//================================================================================================================
void setup() {
  // PC用シリアルの設定
  Serial.begin(SERIAL_PC_BPS);
  while (!Serial) {
    delay(1);
  }
  delay(100);

  // ピンモードの設定
  pinMode(PIN_ERR_LED, OUTPUT); // エラー通知用LED

  // コンデンサの充電時間として待機
  mrd_msg_charging(CHARGE_TIME);

  // 起動メッセージ(バージョン, PC-USB,SPI0,i2c0のスピード)
  mrd_msg_lite_hello();

  // サーボ値の初期設定
  sv.num_max =
      max(max_used_index(IDL_MT, IDL_MAX), max_used_index(IDR_MT, IDR_MAX)); // サーボ処理回数
  for (int i = 0; i <= sv.num_max; i++) { // configで設定した値を反映させる
    sv.idl_mount[i] = IDL_MT[i];
    sv.idr_mount[i] = IDR_MT[i];
    sv.idl_cw[i] = IDL_CW[i];
    sv.idr_cw[i] = IDR_CW[i];
    sv.idl_trim[i] = IDL_TRIM[i];
    sv.idr_trim[i] = IDR_TRIM[i];
  };

  // サーボUARTの通信速度の表示
  mrd_msg_lite_servo_bps();

  // サーボ用UART設定
  mrd_servos_begin(L, MOUNT_L_SERVO_TYPE); // サーボモータの通信初期設定. Serial2
  mrd_servos_begin(R, MOUNT_R_SERVO_TYPE); // サーボモータの通信初期設定. Serial3

  // マウントされたサーボIDの表示
  mrd_msg_lite_servo_mounts();

  // EEPROMの開始, ダンプ表示
  mrd_eeprom_init(EEPROM_SIZE);                                   // EEPROMの初期化
  mrd_eeprom_dump_at_boot(EEPROM_DUMP, EEPROM_FORMAT);            // 内容のダンプ表示
  mrd_eeprom_write_read_check(mrd_eeprom_make_data_from_config(), // EEPROMのリードライトテスト
                              CHECK_EEPROM_RW, EEPROM_PROTECT, EEPROM_FORMAT);

  // SDカードの初期設定とチェック
  mrd_sd_init(MOUNT_SD, PIN_CHIPSELECT_SD);
  mrd_sd_check(MOUNT_SD, PIN_CHIPSELECT_SD, CHECK_SD_RW);

  // I2Cの初期化と開始
  mrd_wire0_setup(BNO055_AHRS, I2C0_SPEED, ahrs, PIN_I2C0_SDA, PIN_I2C0_SCL);

  // I2Cスレッドの開始
  if (MOUNT_IMUAHRS == BNO055_AHRS) {
    xTaskCreatePinnedToCore(mrd_wire0_Core0_bno055_r, "Core0_bno055_r", 4096, NULL, 10, &thp[0], 0);
    Serial.println("Core0 thread for BNO055 start.");
    delay(10);
  }

  // WiFiの初期化と開始
  mrd_msg_esp_wifi();
  init_wifi(WIFI_AP_SSID, WIFI_AP_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    // https://www.arduino.cc/en/Reference/WiFiStatus 戻り値一覧
    delay(1); // 接続が完了するまでループで待つ
  }

  // wifiのIP表示
  mrd_msg_esp_ip(MODE_FIXED_IP);

  // UDP通信の開始
  udp.begin(UDP_RESV_PORT);

  // JOYPADの認識
  mrd_msg_mounted_pad();

  // Bluetoothの初期化
  uint8_t bt_mac[6];
  String self_mac_address = "";
  esp_read_mac(bt_mac, ESP_MAC_BT); // ESP32自身のBluetoothMacアドレスを表示
  self_mac_address = String(bt_mac[0], HEX) + ":" + String(bt_mac[1], HEX) + ":" +
                     String(bt_mac[2], HEX) + ":" + String(bt_mac[3], HEX) + ":" +
                     String(bt_mac[4], HEX) + ":" + String(bt_mac[5], HEX);
  Serial.print("ESP32's Bluetooth Mac Address => " + self_mac_address);
  Serial.println();

  // UDP開始用ダミーデータの生成
  s_udp_meridim.sval[MRD_MASTER] = 90;
  s_udp_meridim.sval[MRD_CKSM] = mrd.cksm_val(s_udp_meridim.sval, MRDM_LEN);
  r_udp_meridim.sval[MRD_MASTER] = 90;
  r_udp_meridim.sval[MRD_CKSM] = mrd.cksm_val(r_udp_meridim.sval, MRDM_LEN);

  // タイマーの設定
  timer_semaphore = xSemaphoreCreateBinary(); // セマフォの作成
  timer = timerBegin(0, 80, true); // タイマーの設定（1つ目のタイマーを使用, 分周比80）
  timerAttachInterrupt(timer, &frame_timer, true); // frame_timer関数をタイマーの割り込みに登録
  timerAlarmWrite(timer, FRAME_DURATION * 1000, true); // タイマーを10msごとにトリガー
  timerAlarmEnable(timer);                             // タイマーを開始

  // 開始のシリアル表示
  mrd_msg_lite_flow_start();
  // タイマーの初期化
  count_frame = 0;
  portENTER_CRITICAL(&timer_mux);
  count_timer = 0;
  portEXIT_CRITICAL(&timer_mux);
}

//================================================================================================================
// MAIN LOOP
//================================================================================================================
void loop() {
  //------------------------------------------------------------------------------------
  //  [ 1 ] UDP送信
  //------------------------------------------------------------------------------------
  mrd.monitor_check_flow("[1]", monitor.flow); // デバグ用フロー表示

  // @[1-1] UDP送信の実行
  if (MODE_UDP_SEND) // 設定でUDPの送信を行うかどうか
  {
    udp_send(s_udp_meridim.bval, MRDM_BYTE);
    flg.udp_rcvd = false;
  }

  //------------------------------------------------------------------------------------
  //  [ 2 ] UDP受信
  //------------------------------------------------------------------------------------
  mrd.monitor_check_flow("[2]", monitor.flow); // デバグ用フロー表示

  // @[2-1] UDPの受信待ち受けループ
  if (MODE_UDP_RECEIVE) // UDPの受信を行うかどうか
  {
    unsigned long startMillis = millis();
    flg.udp_rcvd = false;
    while (!flg.udp_rcvd) {
      if (udp_receive(r_udp_meridim.bval, MRDM_BYTE)) // 受信確認
      {
        flg.udp_rcvd = true;
      }

      // ※パッシブモードを加えること
      unsigned long currentMillis = millis();
      if (currentMillis - startMillis >= UDP_TIMEOUT) // タイムアウト抜け
      {
        Serial.println("UDP timeout");
        flg.udp_rcvd = false;
        break;
      }
      delay(1);
    }
  }

  // @[2-2] チェックサムを確認
  if (mrd.cksm_rslt(r_udp_meridim.sval, MRDM_LEN)) // Check sum OK!
  {
    mrd.monitor_check_flow("CsOK", monitor.flow); // デバグ用フロー表示

    // @[2-3] UDP受信配列から UDP送信配列にデータを転写
    memcpy(s_udp_meridim.bval, r_udp_meridim.bval, MRDM_LEN * 2);

    // @[2-4a] エラーフラグ14番(ESP32のPCからのUDP受信エラー検出)をサゲる.
    s_udp_meridim.bval[MRD_ERR_u] &= B10111111;
  } else // チェックサムがNGならバッファから転記せず前回のデータを使用する
  {
    // @[2-4b] エラーフラグ14番(ESP32のPCからのUDP受信エラー検出)をアゲる.
    err.pc_esp++;
    s_udp_meridim.bval[MRD_ERR_u] |=
        B01000000; // エラーフラグ14番(ESP32のPCからのUDP受信エラー検出)をオン
    mrd.monitor_check_flow("CsErr*", monitor.flow); // デバグ用フロー表示
  }

  // @[2-5] シーケンス番号チェック
  mrdsq.r_expect = mrd.seq_predict_num(mrdsq.r_expect); // シーケンス番号予想値の生成

  // @[2-6] シーケンス番号のシリアルモニタ表示
  mrd_msg_seq_number(mrdsq.r_expect, int(s_udp_meridim.usval[MRD_SEQENTIAL]), monitor.seq_num);

  if (mrd.seq_compare_nums(mrdsq.r_expect, int(s_udp_meridim.usval[MRD_SEQENTIAL]))) {
    s_udp_meridim.bval[MRD_ERR_u] &= B11111011; // [MRD_ERR] 10番bit[ESP受信のスキップ検出]をサゲる.
    flg.meridim_rcvd = true;                    // Meridim受信成功フラグをアゲる.
  } else { // 受信シーケンス番号の値が予想と違ったら
    mrdsq.r_expect = int(s_udp_meridim.usval[MRD_SEQENTIAL]); // 現在の受信値を予想結果としてキープ
    s_udp_meridim.bval[MRD_ERR_u] |= B00000100; // Meridim[MRD_ERR]
                                                // 10番ビット[ESP受信のスキップ検出]をアゲる.
    err.esp_skip++;
    flg.meridim_rcvd = false; // Meridim受信成功フラグをサゲる.
  }

  //------------------------------------------------------------------------------------
  //  [ 3 ] コマンド実行1
  //------------------------------------------------------------------------------------
  mrd.monitor_check_flow("[3]", monitor.flow); // デバグ用フロー表示

  // @[3-1] MastarCommand group1 の処理
  execute_master_command_1(s_udp_meridim, flg.meridim_rcvd);

  //------------------------------------------------------------------------------------
  //  [ 4 ] センサ読み込み
  //------------------------------------------------------------------------------------
  mrd.monitor_check_flow("[4]", monitor.flow); // デバグ用フロー表示

  // @[4-1] センサ値のMeridimへの転記
  mrd_wire0_copy_ahrs_data(BNO055_AHRS, s_udp_meridim.sval, ahrs.read);

  //------------------------------------------------------------------------------------
  //  [ 5 ] リモコン読み込み
  //------------------------------------------------------------------------------------
  mrd.monitor_check_flow("[5]", monitor.flow); // デバグ用フロー表示

  // @[5-1] コントロールパッド受信値の転記
  mrd_pad_reader(MOUNT_PAD, PAD_INTERVAL, PAD_BUTTON_MARGE, monitor.pad);

  //------------------------------------------------------------------------------------
  //  [ 6 ] コマンド実行2
  //------------------------------------------------------------------------------------
  mrd.monitor_check_flow("[6]", monitor.flow); // デバグ用フロー表示

  // @[6-1] MastarCommand group2 の処理
  execute_master_command_2(s_udp_meridim, flg.meridim_rcvd);

  //------------------------------------------------------------------------------------
  //  [ 7 ] ESP32内部で位置制御する場合の処理
  //------------------------------------------------------------------------------------
  mrd.monitor_check_flow("[7]", monitor.flow); // デバグ用フロー表示

  // @[7-1] 前回のラストに読み込んだサーボ位置をサーボ配列に書き込む
  for (int i = 0; i <= sv.num_max; i++) {
    sv.idl_tgt_past[i] = sv.idl_tgt[i];                    // 前回のdegreeをキープ
    sv.idr_tgt_past[i] = sv.idr_tgt[i];                    // 前回のdegreeをキープ
    sv.idl_tgt[i] = s_udp_meridim.sval[i * 2 + 21] * 0.01; // 受信したdegreeを格納
    sv.idr_tgt[i] = s_udp_meridim.sval[i * 2 + 51] * 0.01; // 受信したdegreeを格納
  }

  // @[7-2] 各種処理

  //------------------------------------------------------------------------------------
  //  [ 8 ] サーボ動作の実行
  //------------------------------------------------------------------------------------
  mrd.monitor_check_flow("[8]", monitor.flow); // デバグ用フロー表示

  // @[8-1] サーボ受信値の処理
  if (!MODE_ESP32_STDALONE) { // サーボ処理を行うかどうか
    mrd_servos_drive_lite(s_udp_meridim, MOUNT_L_SERVO_TYPE, MOUNT_R_SERVO_TYPE, sv); // サーボ動作を実行する
  } else {
    sv.idl_tgt[0] = sin(tmr.loop_count * M_PI / 180.0) *
                    30; // ボード単体動作モードの場合はサーボの戻り値を調べず,
                        // L0番サーボ値として+-30度のサインカーブの値を返す
  }

  //------------------------------------------------------------------------------------
  //  [ 9 ] サーボ受信値の処理
  //------------------------------------------------------------------------------------
  mrd.monitor_check_flow("[9]", monitor.flow); // デバグ用フロー表示

  // @[9-1] サーボIDごとにの現在位置もしくは計算結果を配列に格納
  for (int i = 0; i <= sv.num_max; i++) {
    // 最新のサーボ角度をdegreeで格納
    s_udp_meridim.sval[i * 2 + 21] = mrd.float2HfShort(sv.idl_tgt[i]);
    s_udp_meridim.sval[i * 2 + 51] = mrd.float2HfShort(sv.idr_tgt[i]);
  }

  //------------------------------------------------------------------------------------
  //  [ 10 ] エラーリポートの作成
  //------------------------------------------------------------------------------------
  mrd.monitor_check_flow("[10]", monitor.flow); // デバグ用フロー表示

  // @[10-1] 全エラーの表示
  mrd_msg_all_err(err, monitor.all_err);

  //------------------------------------------------------------------------------------
  //  [ 11 ] UDP送信信号作成
  //------------------------------------------------------------------------------------
  mrd.monitor_check_flow("[11]", monitor.flow); // デバグ用フロー表示

  // @[11-1] フレームスキップ検出用のカウントをカウントアップして送信用に格納
  mrdsq.s_increment = mrd.seq_increase_num(mrdsq.s_increment);
  s_udp_meridim.usval[1] = mrdsq.s_increment;

  // @[11-2] チェックサムを計算して格納
  s_udp_meridim.sval[MRD_CKSM] = mrd.cksm_val(s_udp_meridim.sval, MRDM_LEN);

  //------------------------------------------------------------------------------------
  //   [ 12 ] フレーム終端処理
  //------------------------------------------------------------------------------------
  mrd.monitor_check_flow("[12]", monitor.flow); // 動作チェック用シリアル表示

  // @[12-1] count_timerがcount_frameに追いつくまで待機
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

  // @[12-2] 必要に応じてフレームの遅延累積時間frameDelayをリセット
  if (flg.frame_timer_reset) {
    portENTER_CRITICAL(&timer_mux);
    count_frame = count_timer;
    portEXIT_CRITICAL(&timer_mux);
  }

  mrd.monitor_check_flow("\n", monitor.flow); // 動作チェック用シリアル表示
}

//================================================================================================================
//  関数各種
//================================================================================================================

/// @brief 配列の中で0以外が入っている最大のIndexを求めます.
/// @param a_arr 配列
/// @param a_size 配列の長さ
/// @return 0以外が入っている最大のIndex. すべて0の場合は0を反す.
int max_used_index(const int a_arr[], int a_size) {
  int maxIndex_tmp = 0;
  for (int i = 0; i < a_size; ++i) {
    if (a_arr[i] != 0) {
      maxIndex_tmp = i;
    }
  }
  return maxIndex_tmp;
}

//================================================================================================================
//  コマンド処理
//================================================================================================================

/// @brief Master Commandの第1群を実行する. 受信コマンドに基づき, 異なる処理を行う.
/// @param a_meridim 実行したいコマンドの入ったMeridim配列を渡す.
/// @param a_flg_exe Meridimの受信成功判定フラグを渡す.
/// @return コマンドを実行した場合はtrue, しなかった場合はfalseを返す.
bool execute_master_command_1(Meridim90Union a_meridim, bool a_flg_exe) {
  if (!a_flg_exe) {
    return false;
  }

  // コマンド[90]: 1~999は MeridimのLength. デフォルトは90

  // コマンド:MCMD_CLEAR_SERVO_ERROR_ID (10004) 通信エラーサーボIDのクリア
  if (a_meridim.sval[MRD_MASTER] == MCMD_CLEAR_SERVO_ERROR_ID) {
    r_udp_meridim.bval[MRD_ERR_l] = 0;
    s_udp_meridim.bval[MRD_ERR_l] = 0;
    for (int i = 0; i < IDL_MAX; i++) {
      sv.idl_err[i] = 0;
    }
    for (int i = 0; i < IDR_MAX; i++) {
      sv.idr_err[i] = 0;
    }
    Serial.println("Servo Error ID reset.");
    return true;
  }

  // コマンド:MCMD_BOARD_TRANSMIT_ACTIVE (10005) UDP受信の通信周期制御をボード側主導に（デフォルト）
  if (a_meridim.sval[MRD_MASTER] == MCMD_BOARD_TRANSMIT_ACTIVE) {
    flg.udp_board_passive = false; // UDP送信をアクティブモードに
    flg.frame_timer_reset = true; // フレームの管理時計をリセットフラグをセット
    return true;
  }

  // コマンド:MCMD_ENTER_EEPROM_WRITE_MODE (10009) EEPROMの書き込みモードスタート
  if (a_meridim.sval[MRD_MASTER] == MCMD_ENTER_EEPROM_WRITE_MODE) {
    flg.eeprom_write_mode = true; // 書き込みモードのフラグをセット
    flg.frame_timer_reset = true; // フレームの管理時計をリセットフラグをセット
    return true;
  }
  return false;
}

/// @brief Master Commandの第2群を実行する. 受信コマンドに基づき, 異なる処理を行う.
/// @param a_meridim 実行したいコマンドの入ったMeridim配列を渡す.
/// @param a_flg_exe Meridimの受信成功判定フラグを渡す.
/// @return コマンドを実行した場合はtrue, しなかった場合はfalseを返す.
bool execute_master_command_2(Meridim90Union a_meridim, bool a_flg_exe) {
  if (!a_flg_exe) {
    return false;
  }
  // コマンド[90]: 1~999は MeridimのLength. デフォルトは90

  // コマンド:[0] 全サーボ脱力
  if (a_meridim.sval[MRD_MASTER] == 0) {
    mrd_servos_all_off(s_udp_meridim);
    return true;
  }

  // コマンド:[1] サーボオン 通常動作

  // コマンド:MCMD_UPDATE_YAW_CENTER (10002) IMU/AHRSのヨー軸リセット
  if (a_meridim.sval[MRD_MASTER] == MCMD_UPDATE_YAW_CENTER) {
    ahrs.yaw_origin = ahrs.yaw_source;
    Serial.println("cmd: yaw reset.");
    return true;
  }

  // コマンド:MCMD_ENTER_TRIM_MODE (10003) トリムモードに入る（既存のものは廃止し, 検討中）

  // コマンド:MCMD_BOARD_TRANSMIT_PASSIVE (10006) UDP受信の通信周期制御をPC側主導に（SSH的な動作）
  if (a_meridim.sval[MRD_MASTER] == MCMD_BOARD_TRANSMIT_PASSIVE) {
    flg.udp_board_passive = true; // UDP送信をパッシブモードに
    flg.frame_timer_reset = true; // フレームの管理時計をリセットフラグをセット
    return true;
  }

  // コマンド:MCMD_RESET_MRD_TIMER) (10007) フレーム管理時計tmr.mrd_milを現在時刻にリセット
  if (a_meridim.sval[MRD_MASTER] == MCMD_RESET_MRD_TIMER) {
    flg.frame_timer_reset = true; // フレームの管理時計をリセットフラグをセット
    return true;
  }

  // コマンド:MCMD_STOP_BOARD_DURING (10008) ボードの末端処理を指定時間だけ止める.
  if (a_meridim.sval[MRD_MASTER] == MCMD_STOP_BOARD_DURING) {
    flg.stop_board_during = true; // ボードの処理停止フラグをセット
    // ボードの末端処理をmeridim[2]ミリ秒だけ止める.
    Serial.print("Stop ESP32's processing during ");
    Serial.print(int(a_meridim.sval[MRD_STOP_FRAMES_MS]));
    Serial.println(" ms.");
    for (int i = 0; i < int(a_meridim.sval[MRD_STOP_FRAMES_MS]); i++) {
      delay(1);
    }
    flg.stop_board_during = false; // ボードの処理停止フラグをクリア
    flg.frame_timer_reset = true; // フレームの管理時計をリセットフラグをセット
    return true;
  }
  return false;
}
