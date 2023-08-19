// Meridan Board -LITE-
// ESP32用スクリプト　20230818版
#define VERSION "Hello, This is Meridian_LITE_20230818." // バージョン表示

// [ 概 要 ] ///////////////////////////////////////////////////////////////////////
// MeridianはMeridim90というサーボやセンサーの状態が格納されたデータ配列を, PCと100Hzで共有する.
// ハードウェアの制限上, サーボ値の返信は取りこぼしなどあるが, 取りこぼす直前のデータを使うことで擬似的に安定させている.
// ICSサーボの返信データはサーボ由来の揺らぎがあるため現状では参考値と考えるのがよく,サーボ値を指示して動作させることが基本的な利用法となる.

// [ 利 用 プ ラ ッ ト フ ォ ー ム ] //////////////////////////////////////////////////
//  ESP32 DevkitC + PlatformIOで使うことを想定
//  PlatformIO上でのESP32のボードバージョンは3.5.0が良い.（4系はうまく動かないらしい）
//  Serial1を使うには設定を変更する必要あり. 参考 → https://qiita.com/Ninagawa_Izumi/items/8ce2d55728fd5973087d

// [ デ フ ォ ル ト の ハ ー ド ウ ェ ア ] /////////////////////////////////////////////
// 100Hz通信で安定動作.
// デフォルトの環境設定は
// ① Meridian Board -LITE- に Espressif ESP32DevKitCを搭載.
// ② ICSサーボL,R 11個ずつ合計22軸分を接続
// ③ リモコン KRC-5FH + KRR-5FH
// ④ 9軸センサ BNO005を搭載
// ⑤ SDカード(SPIに接続)は不搭載
//
// 上記を搭載している場合,
// サーボについてはconfig.h内にて下記の初期設定をしておく.
// IDL_MT0 - IDR_MT14     : 各サーボのマウントありなし(デフォルトはKHR - 3HVの22軸設定.)
// IDL_CW0 - IDR_CW14     : 各サーボの回転方向の正負補正
// IDL_TRIM0 - IDR_TRIM14 : 直立時の各サーボのデフォルト値
// その他については,config.h内の SD_MOUNT,MOUNT_IMUAHRS,MOUNT_JOYPAD などをそれぞれ設定.

// [ WIFI設定 ] ////////////////////////////////////////////////////////////////////
// サーボについてはconfig.h内の /* Wifiアクセスポイントの設定 */ にて
// WIFI_AP_SSID, WIFI_AP_PASS, WIFI_SEND_IP を設定する.
// PCのIPアドレスはPC側で調べる.
// ESP32側のIPアドレス, BluetoothMACアドレスはEPS32の起動時にシリアルモニタに表示される.

// [ 更 新 履 歴 ] /////////////////////////////////////////////////////////////////
// 20230617 20220703版をベースに, 全面改訂.
// 20230617 公式ライブラリ<Meridian.h> バージョンxx に対応.
// 20230617 変数名や関数名をMeridian TWINと共通化.
// 20230617 サーボ角の扱いの基本型をdegree値に変更.
// 20230617 config.hとmain.hを作成.

//================================================================================================================
//---- 初 期 設 定  -----------------------------------------------------------------------------------------------
//================================================================================================================

//////// モジュール導入 ////////////////
/* ライブラリ導入 */
#include <Arduino.h>
#include <Meridian.h>           // Meridianのライブラリ導入
#include <WiFi.h>               // WiFi通信用ライブラリ
#include <WiFiUdp.h>            // UDP通信用ライブラリ
#include <IcsHardSerialClass.h> // KONDOサーボのライブラリ
#include <Wire.h>               // I2C通信用ライブラリ
#include <Adafruit_BNO055.h>    // 9軸センサBNO055用のライブラリ
#include <SPI.h>                // SPI自体は使用しないがBNO055に必要
#include <SD.h>                 // SDカード用のライブラリ
#include <map>                  // 辞書型配列用のライブラリ
/* コンフィグファイルの読み込み */
#include "config.h"
/* ヘッダファイルの読み込み */
#include "main.h"

//////// インスタンス関連 //////////////
MERIDIANFLOW::Meridian mrd;                                              // ライブラリのクラスを mrdという名前でインスタンス化
IcsHardSerialClass krs_L(&Serial1, PIN_EN_L, ICS_BAUDRATE, ICS_TIMEOUT); // サーボL系統UARTの設定（TX27,RX32,EN33）
IcsHardSerialClass krs_R(&Serial2, PIN_EN_R, ICS_BAUDRATE, ICS_TIMEOUT); // サーボR系統UARTの設定（TX17,RX16,EN4）
WiFiUDP udp;

//////// Meridim配列定義関連 //////////
const int MSG_BUFF = MSG_SIZE * 2;     // Meridim配列の長さ（byte換算）
const int MSG_ERR = MSG_SIZE - 2;      // エラーフラグの格納場所（配列の末尾から2つめ）
const int MSG_ERR_u = MSG_ERR * 2 + 1; // エラーフラグの格納場所（上位8ビット）
const int MSG_ERR_l = MSG_ERR * 2;     // エラーフラグの格納場所（下位8ビット）
const int MSG_CKSM = MSG_SIZE - 1;     // チェックサムの格納場所（配列の末尾）
typedef union                          // Meridim配列用の共用体の設定
{
  short sval[MSG_SIZE + 4];           // short型でデフォルト90個の配列データを持つ
  unsigned short usval[MSG_SIZE + 2]; // 上記のunsigned short型
  uint8_t bval[MSG_BUFF + 4];         // 1バイト単位でデフォルト180個の配列データを持つ
} UnionData;
UnionData s_udp_meridim = {0}; // UDP送信用共用体のインスタンスを宣言
UnionData r_udp_meridim = {0}; // UDP受信用共用体のインスタンスを宣言
// UnionData pad_udp_meridim = {0}; // UDP送信用共用体のインスタンスを宣言

//////// システム関連 /////////////////
TaskHandle_t thp[4];                                           // マルチスレッドのタスクハンドル格納用
File myFile;                                                   // SDカード用
int servo_num_max = max(MOUNT_SERVO_NUM_L, MOUNT_SERVO_NUM_R); // サーボ送受信のループ処理数（L系R系で多い方）

//////// フラグ関連 ///////////////////
bool udp_rsvd_flag = 0; // UDPの受信終了フラグ

//////// タイマー関連 /////////////////
long frame_ms = FRAME_DURATION;  // 1フレームあたりの単位時間(ms)
int frame_count = 0;             // サイン計算用の変数
int frame_count_diff = 2;        // サインカーブ動作などのフレームカウントをいくつずつ進めるか
int frame_count_max = 360000;    // フレームカウントの最大値
int joypad_frame_count = 0;      // JOYPADのデータを読みに行くためのフレームカウント
long mrd_t_mil = (long)millis(); // フレーム管理時計の時刻 Meridian Time.
long now_t_mil = (long)millis(); // 現在時刻をミリ秒で取得
long now_t_mic = (long)micros(); // 現在時刻をマイクロ秒で取得
int mrd_seq_s_increment = 0;     // フレーム毎に0-59999をカウントし、送信
int mrd_seq_r_expect = 0;        // フレーム毎に0-59999をカウントし、受信値と比較
int mrd_seq_r = 0;               // 今フレームに受信したframe_syncを格納

//////// エラー管理 ///////////////////
int err_pc_esp = 0; // ESP32の受信エラー（PCからのUDP）

//////// モード切り替え関連 ////////////
// config.hにて設定

//////// リモコン関連 /////////////////
typedef union
{
  short sval[4];       // short型で4個の配列データを持つ
  uint16_t usval[4];   // 上記のunsigned short型
  int8_t bval[8];      // 上記のbyte型
  uint8_t ubval[8];    // 上記のunsigned byte型
  uint64_t ui64val[1]; // 上記のunsigned int16型
                       // button, pad_stick_L_x:pad_stick_L_y,
                       // pad_stick_R_x:pad_stick_R_y, pad_L2_val:pad_R2_val
} UnionPad;
UnionPad pad_array = {0}; // リモコン値格納用の配列
unsigned short pad_btn = 0;
int pad_stick_R = 0;
int pad_stick_R_x = 0;
int pad_stick_R_y = 0;
int pad_stick_L = 0;
int pad_stick_L_x = 0;
int pad_stick_L_y = 0;
int pad_stick_V = 0;
int pad_R2_val = 0;
int pad_L2_val = 0;

//////// センサー関連 /////////////////
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
float bno055_read[16]; // bno055のデータの一時保存
float yaw_center = 0;  // ヨー軸の原点セット用

//////// サーボボジション関連 //////////
/* サーボのポジション用配列.*/
int s_servo_pos_L[15] = {0}; // 15要素 100倍したdegree値
int s_servo_pos_R[15] = {0}; // 15要素 100倍したdegree値

/* サーボのエラーカウンタ配列.*/
int idl_err[15] = {0}; // 15要素
int idr_err[15] = {0}; // 15要素

/* 各サーボのポジション値(degree) */
float idl_tgt[15] = {0};      // L系統の目標値
float idr_tgt[15] = {0};      // R系統の目標値
float idl_tgt_past[15] = {0}; // L系統の前回の値
float idr_tgt_past[15] = {0}; // R系統の前回の値

/* 各サーボのマウントありなし */
int idl_mount[15] = {IDL_MT0, IDL_MT1, IDL_MT2, IDL_MT3, IDL_MT4, IDL_MT5, IDL_MT6, IDL_MT7, IDL_MT8, IDL_MT9, IDL_MT10, IDL_MT11, IDL_MT12, IDL_MT13, IDL_MT14}; // L系統
int idr_mount[15] = {IDR_MT0, IDR_MT1, IDR_MT2, IDR_MT3, IDR_MT4, IDR_MT5, IDR_MT6, IDR_MT7, IDR_MT8, IDR_MT9, IDR_MT10, IDR_MT11, IDR_MT12, IDR_MT13, IDR_MT14}; // R系統
int id3_mount[15] = {0};                                                                                                                                          // 3系統

/* 各サーボの回転方向順逆補正用 */
int idl_cw[15] = {IDL_CW0, IDL_CW1, IDL_CW2, IDL_CW3, IDL_CW4, IDL_CW5, IDL_CW6, IDL_CW7, IDL_CW8, IDL_CW9, IDL_CW10, IDL_CW11, IDL_CW12, IDL_CW13, IDL_CW14}; // L系統
int idr_cw[15] = {IDR_CW0, IDR_CW1, IDR_CW2, IDR_CW3, IDR_CW4, IDR_CW5, IDR_CW6, IDR_CW7, IDR_CW8, IDR_CW9, IDR_CW10, IDR_CW11, IDR_CW12, IDR_CW13, IDR_CW14}; // R系統

/* 各サーボの直立ポーズトリム値(degree) */
float idl_trim[15] = {IDL_TRIM0, IDL_TRIM1, IDL_TRIM2, IDL_TRIM3, IDL_TRIM4, IDL_TRIM5, IDL_TRIM6, IDL_TRIM7, IDL_TRIM8, IDL_TRIM9, IDL_TRIM10, IDL_TRIM11, IDL_TRIM12, IDL_TRIM13, IDL_TRIM14}; // L系統
float idr_trim[15] = {IDR_TRIM0, IDR_TRIM1, IDR_TRIM2, IDR_TRIM3, IDR_TRIM4, IDR_TRIM5, IDR_TRIM6, IDR_TRIM7, IDR_TRIM8, IDR_TRIM9, IDR_TRIM10, IDR_TRIM11, IDR_TRIM12, IDR_TRIM13, IDR_TRIM14}; // R系統

//////// 計算用変数 //////////////////
float n; // 計算用
int k;   // 各サーボの計算用変数

//================================================================================================================
//---- S E T  U P -----------------------------------------------------------------------------------------------
//================================================================================================================
void setup()
{
  /* ピンモードの設定 */
  pinMode(ERR_LED, OUTPUT); // エラー通知用LED

  /* PC用シリアルの設定 */
  Serial.begin(SERIAL_PC_BPS);

  /* サーボモーター用シリアルの設定 */
  krs_L.begin();
  krs_R.begin();
  delay(200);

  /* 起動メッセージ1 */
  mrd.print_esp_hello_start(VERSION, String(SERIAL_PC_BPS), WIFI_AP_SSID);

  /* WiFiの初期化と開始 */
  init_wifi(WIFI_AP_SSID, WIFI_AP_PASS);

  /* 起動メッセージ2 */
  mrd.print_esp_hello_ip(WIFI_SEND_IP, WiFi.localIP(), FIXED_IP_ADDR, MODE_FIXED_IP);

  /* UDP通信の開始 */
  udp.begin(UDP_RESV_PORT);

  /* I2Cの開始 */
  Wire.begin(22, 21);

  /* センサの初期化 */
  init_imuahrs(MOUNT_IMUAHRS);

  /* スレッドの開始 */
  if (MOUNT_IMUAHRS == 3)
  {
    xTaskCreatePinnedToCore(Core1_bno055_r, "Core1_bno055_r", 4096, NULL, 10, &thp[0], 1);
    Serial.println("Core1 Slead for bno055 start.");
    delay(10); // センサー用スレッド
  }

  /* SDカードの初期設定とチェック */
  check_sd();

  /* マウントされたサーボIDの表示 */
  // mrd.print_servo_mounts(idl_mount, idr_mount, id3_mount);
  mrd.print_servo_mounts(idl_mount, idr_mount, id3_mount);

  /* JOYPADの認識 */
  mrd.print_controlpad(MOUNT_JOYPAD, JOYPAD_POLLING);

  /* Bluetoothの初期化 */
  uint8_t bt_mac[6];
  String self_mac_address = "";
  esp_read_mac(bt_mac, ESP_MAC_BT); // ESP32自身のBluetoothMacアドレスを表示
  self_mac_address = String(bt_mac[0], HEX) + ":" + String(bt_mac[1], HEX) + ":" + String(bt_mac[2], HEX) + ":" + String(bt_mac[3], HEX) + ":" + String(bt_mac[4], HEX) + ":" + String(bt_mac[5], HEX);
  Serial.print("ESP32's Bluetooth Mac Address is => " + self_mac_address);
  Serial.println();
  delay(1000);

  /* タイマーの調整と開始のシリアル表示 */
  mrd_t_mil += (long)millis() + 10;                                   // 周期管理用のMeridianTimeをリセット
  Serial.println("-) Meridian -LITE- system on ESP32 now flows. (-"); //

  /* UDP開始用のダミーデータの生成 */
  if (UDP_SEND) // 設定でUDPの送信を行うかどうか決定
  {
    s_udp_meridim.sval[MSG_CKSM] = mrd.cksm_val(s_udp_meridim.sval, MSG_SIZE);
    sendUDP();
    mrd.monitor_check_flow("[start]", MONITOR_FLOW);
  }
}

//================================================================================================================
//---- M A I N  L O O P -----------------------------------------------------------------------------------------
//================================================================================================================
void loop()
{

  //////// < 1 > U D P 受 信 ///////////////////////////////////////////////////////
  // @ [1-1] UDP受信の実行 もしデータパケットが来ていれば受信する
  if (UDP_RESEIVE) // UDPの受信を行うかどうか
  {
    receiveUDP(); // UDPを受信
    if (udp_rsvd_flag)
    {
      mrd.monitor_check_flow("[Rsvd]", MONITOR_FLOW); // デバグ用フロー表示
      udp_rsvd_flag = 0;
    }
  }
  mrd.monitor_check_flow("[1]", MONITOR_FLOW); // デバグ用フロー表示

  // 　→ ここでr_udp_meridim.sval に受信したMeridim配列が入っている状態。

  // @ [1-2] チェックサムを確認
  if (mrd.cksm_rslt(r_udp_meridim.sval, MSG_SIZE)) // Check sum OK!
  {
    mrd.monitor_check_flow("[CSok]", MONITOR_FLOW); // デバグ用フロー表示

    // @ [1-3] UDP受信配列から UDP送信配列にデータを転写
    memcpy(s_udp_meridim.bval, r_udp_meridim.bval, MSG_SIZE * 2);

    // @ [1-4] エラーフラグ14番(ESP32のPCからのUDP受信エラー検出)をオフ
    s_udp_meridim.bval[MSG_ERR_u] &= B10111111;

    //////// < 2 > リ モ コ ン 受 信 ///////////////////////////////////////////////////
    // @ [2-1] コントロールパッド受信値の転記
    if (MOUNT_JOYPAD != 0)
    {
      pad_array.ui64val[0] = joypad_read(MOUNT_JOYPAD, pad_array.ui64val[0], JOYPAD_POLLING, JOYPAD_REFRESH);
      s_udp_meridim.usval[MRD_CONTROL_BUTTONS] = pad_array.usval[0];
      if (MONITOR_JOYPAD)
      {
        monitor_joypad(pad_array.usval);
      }
      mrd.monitor_check_flow("[2]", MONITOR_FLOW); // デバグ用フロー表示
    }

    //////// < 3 > 受 信 コ マ ン ド に 基 づ く 制 御 処 理 /////////////////////////////
    // @ [3-1] AHRSのヨー軸リセット
    if (r_udp_meridim.sval[0] == MCMD_UPDATE_YAW_CENTER)
    {
      setyawcenter();
    }
    mrd.monitor_check_flow("[3]", MONITOR_FLOW); // デバグ用フロー表示

    //////// < 4 > E S P 内 部 で 位 置 制 御 す る 場 合 の 処 理 ///////////////////////
    /* 現在はとくに設定なし */
    mrd.monitor_check_flow("[4]", MONITOR_FLOW); // デバグ用フロー表示

    //////// < 5 > サ ー ボ 動 作 の 実 行 /////////////////////////////////////////////
    // @ [5-1] 受信したサーボ位置をサーボ配列に書き込む
    for (int i = 0; i < servo_num_max; i++)
    {
      idl_tgt_past[i] = idl_tgt[i];                       // 前回のdegreeをキープ
      idr_tgt_past[i] = idr_tgt[i];                       // 前回のdegreeをキープ
      idl_tgt[i] = s_udp_meridim.sval[i * 2 + 21] * 0.01; // 受信したdegreeを格納
      idr_tgt[i] = s_udp_meridim.sval[i * 2 + 51] * 0.01; // 受信したdegreeを格納
    }

    // @ [5-2] サーボ受信値の処理
    for (int i = 0; i < servo_num_max; i++) // ICS_L系統の処理
    {                                       // 接続したサーボの数だけ繰り返す。最大は15
      if (idl_mount[i])
      {
        if (r_udp_meridim.sval[(i * 2) + 20] == 1) // 受信配列のサーボコマンドが1ならPos指定
        {
          k = krs_L.setPos(i, mrd.Deg2Krs(idl_tgt[i], idl_trim[i], idl_cw[i]));
          if (k == -1) // サーボからの返信信号を受け取れなかった時は前回の数値のままにする
          {
            k = mrd.Deg2Krs(idl_tgt_past[i], idl_trim[i], idl_cw[i]);
            idl_err[i]++;
            if (idl_err[i] >= SERVO_LOST_ERROR_WAIT)
            {
              s_udp_meridim.bval[MSG_ERR_l] = char(i); // Meridim[MSG_ERR] エラーを出したサーボID（0をID[L00]として[L99]まで）
              mrd.monitor_servo_error("L", i, MONITOR_SERVO_ERR);
            }
          }
          else
          {
            idl_err[i] = 0;
          }
        }
        else // 1以外ならとりあえずサーボを脱力し位置を取得。手持ちの最大は15
        {
          k = krs_L.setFree(i); // サーボからの返信信号を受け取れていれば値を更新
          if (k == -1)          // サーボからの返信信号を受け取れなかった時は前回の数値のままにする
          {
            k = mrd.Deg2Krs(idl_tgt_past[i], idl_trim[i], idl_cw[i]);
            idl_err[i]++;
            if (idl_err[i] >= SERVO_LOST_ERROR_WAIT)
            {
              s_udp_meridim.bval[MSG_ERR_l] = char(i); // Meridim[MSG_ERR] エラーを出したサーボID（0をID[L00]として[L99]まで）
              mrd.monitor_servo_error("L", i, MONITOR_SERVO_ERR);
            }
          }
          else
          {
            idl_err[i] = 0;
          }
        }
        idl_tgt[i] = mrd.Krs2Deg(k, idl_trim[i], idl_cw[i]);
      }
      // delayMicroseconds(2);

      if (idr_mount[i])
      {
        if (r_udp_meridim.sval[(i * 2) + 50] == 1) // 受信配列のサーボコマンドが1ならPos指定
        {
          k = krs_R.setPos(i, mrd.Deg2Krs(idr_tgt[i], idr_trim[i], idr_cw[i]));
          if (k == -1) // サーボからの返信信号を受け取れなかった時は前回の数値のままにする
          {
            k = mrd.Deg2Krs(idr_tgt_past[i], idr_trim[i], idr_cw[i]);
            idr_err[i]++;
            if (idr_err[i] >= SERVO_LOST_ERROR_WAIT)
            {
              s_udp_meridim.bval[MSG_ERR_l] = char(i + 100); // Meridim[MSG_ERR] エラーを出したサーボID（100をID[R00]として[R99]まで）
              mrd.monitor_servo_error("R", i + 100, MONITOR_SERVO_ERR);
            }
          }
          else
          {
            idr_err[i] = 0;
          }
        }
        else // 1以外ならとりあえずサーボを脱力し位置を取得
        {
          k = krs_R.setFree(i);
          if (k == -1) // サーボからの返信信号を受け取れなかった時は前回の数値のままにする
          {
            k = mrd.Deg2Krs(idr_tgt_past[i], idr_trim[i], idr_cw[i]);
            idr_err[i]++;
            if (idr_err[i] >= SERVO_LOST_ERROR_WAIT)
            {
              s_udp_meridim.bval[MSG_ERR_l] = char(i + 100); // Meridim[MSG_ERR] エラーを出したサーボID（100をID[R00]として[R99]まで）
              mrd.monitor_servo_error("R", i + 100, MONITOR_SERVO_ERR);
            }
          }
          else
          {
            idr_err[i] = 0;
          }
        }
        idr_tgt[i] = mrd.Krs2Deg(k, idr_trim[i], idr_cw[i]);
      }
      delayMicroseconds(2);
    }
    mrd.monitor_check_flow("[5]", MONITOR_FLOW); // デバグ用フロー表示
  }
  else // Check sum NG*
  {
    err_pc_esp++;
    s_udp_meridim.bval[MSG_ERR_u] |= B01000000;     // エラーフラグ14番(ESP32のPCからのUDP受信エラー検出)をオン
    mrd.monitor_check_flow("[csNG]", MONITOR_FLOW); // デバグ用フロー表示
  }

  //////// < 6 > サ ー ボ 受 信 値 の 処 理 //////////////////////////////////////////
  // @[6-1] サーボIDごとにの現在位置もしくは計算結果を配列に格納
  for (int i = 0; i < 15; i++)
  {
    // s_udp_meridim.sval[i * 2 + 20] = 0;                              // 仮にここでは各サーボのコマンドを脱力&ポジション指示(0)に設定
    s_udp_meridim.sval[i * 2 + 21] = mrd.float2HfShort(idl_tgt[i]); // 仮にここでは最新のサーボ角度degreeを格納
  }
  for (int i = 0; i < 15; i++)
  {
    // s_udp_meridim.sval[i * 2 + 50] = 0;                              // 仮にここでは各サーボのコマンドを脱力&ポジション指示(0)に設定
    s_udp_meridim.sval[i * 2 + 51] = mrd.float2HfShort(idr_tgt[i]); // 仮にここでは最新のサーボ角度degreeを格納
  }
  mrd.monitor_check_flow("[6]", MONITOR_FLOW); // デバグ用フロー表示

  //////// < 7 > エ ラ ー リ ポ ー ト の 作 成 ///////////////////////////////////////
  // @ [7-1] 通信エラー処理(スキップ検出)
  mrd_seq_r_expect = mrd.predict_seq_num(mrd_seq_r_expect); // シーケンス番号予想値
  // mrd_seq_r_expect = mrd.seq_predict_num(mrd_seq_r_expect); // シーケンス番号予想値
  mrd_seq_r = int(s_udp_meridim.usval[1]); // 受信したシーケンス番号

  if (MONITOR_SEQ)
  {
    Serial.print("exp:revd/");
    Serial.print(mrd_seq_r_expect);
    Serial.print(":");
    Serial.println(mrd_seq_r);
  }

  // if (mrd.seq_compare_nums(mrd_seq_r_expect, mrd_seq_r)) // シーケンス番号が期待通りなら順番どおり受信

  if (mrd.seq_compare_nums(mrd_seq_r_expect, mrd_seq_r)) // シーケンス番号が期待通りなら順番どおり受信
  {
    // Serial.println("BINGO");
    s_udp_meridim.bval[MSG_ERR_u] &= B11111011; // エラーフラグ10番(ESP受信のスキップ検出)をオフ
  }
  else
  {
    // Serial.println("bongo");
    s_udp_meridim.bval[MSG_ERR_u] |= B00000100; // エラーフラグ10番(ESP受信のスキップ検出)をオン
                                                // if (mrd_seq_r == mrd_seq_r_expect - 1)
    //{
    //  同じ値を２回取得した場合には実際はシーケンスが進んだものとして補正（アルゴリズム要検討）
    //  mrd_seq_r_expect = mrd_seq_r + 1;
    //}
    // else
    //{
    mrd_seq_r_expect = mrd_seq_r; // 取りこぼしについては現在の受信値を正解の予測値としてキープ
    //}
  }
  mrd.monitor_check_flow("[7]", MONITOR_FLOW); // デバグ用フロー表示

  //////// < 8 > フ レ ー ム 終 端 処 理 ////////////////////////////////////////////
  // @ [8-1] この時点で１フレーム内に処理が収まっていない時の処理
  now_t_mil = (long)millis(); // 現在時刻を更新
  if (now_t_mil > mrd_t_mil)
  {                               // 現在時刻がフレーム管理時計を超えていたらアラートを出す
    Serial.print("[ERR] delay:"); // シリアルに遅延msを表示
    Serial.println(now_t_mil - mrd_t_mil);
    digitalWrite(ERR_LED, HIGH); // 処理落ちが発生していたらLEDを点灯
  }
  else
  {
    digitalWrite(ERR_LED, LOW); // 処理が収まっていればLEDを消灯
  }

  // @ [8-2] この時点で時間が余っていたら時間消化。時間がオーバーしていたらこの処理を自然と飛ばす。
  now_t_mil = (long)millis();
  now_t_mic = (long)micros(); // 現在時刻を取得
  while ((mrd_t_mil - now_t_mil) >= 1)
  {
    delay(1);
    now_t_mil = (long)millis();
  }
  while (now_t_mic < mrd_t_mil * 1000)
  {
    now_t_mic = (long)micros(); // 現在時刻を取得
  }

  // @ [8-3] フレーム管理時計mercのカウントアップ
  mrd_t_mil = mrd_t_mil + frame_ms;             // フレーム管理時計を1フレーム分進める
  frame_count = frame_count + frame_count_diff; // サインカーブ動作用のフレームカウントをいくつずつ進めるかをここで設定。
  mrd.monitor_check_flow("[8]", MONITOR_FLOW);  // デバグ用フロー表示

  //////// < 9 > U D P 送 信 信 号 作 成 ////////////////////////////////////////////
  // @ [9-1] センサーからの値を送信用に格納
  if (MOUNT_IMUAHRS == 3)
  {
    for (int i = 0; i < 15; i++)
    {
      s_udp_meridim.sval[i] = mrd.float2HfShort(bno055_read[i]); // DMP_ROLL推定値
    }
  }

  // @ [9-2] フレームスキップ検出用のカウントをカウントアップして送信用に格納
  mrd_seq_s_increment = mrd.increase_seq_num(mrd_seq_s_increment);
  // mrd_seq_s_increment = mrd.seq_increase_num(mrd_seq_s_increment);
  s_udp_meridim.usval[1] = mrd_seq_s_increment;

  // @ [9-3] チェックサムを計算して格納
  s_udp_meridim.sval[MSG_CKSM] = mrd.cksm_val(s_udp_meridim.sval, MSG_SIZE);
  mrd.monitor_check_flow("[9]", MONITOR_FLOW); // デバグ用フロー表示

  //////// < 10 > U D P 送 信 //////////////////////////////////////////////////////
  // @ [10-1] UDP送信を実行
  if (UDP_SEND) // 設定でUDPの送信を行うかどうか決定
  {
    sendUDP();
    mrd.monitor_check_flow("[10]\n", MONITOR_FLOW); // デバグ用フロー表示
  }
  delayMicroseconds(5);
}

//================================================================================================================
//---- 関 数 各 種  -----------------------------------------------------------------------------------------------
//================================================================================================================

// +-------------------------------------------------------------------
// | 関数名　　:  setyawcenter()
// +-------------------------------------------------------------------
// | 機能     :  センサーのヨー軸中央値を再セットする
// +------------------------------------------------------------------
void setyawcenter()
{
  yaw_center = bno055_read[14];
}

// +----------------------------------------------------------------------
// | 関数名　　:  receiveUDP()
// +----------------------------------------------------------------------
// | 機能     :  UDP通信の受信パケットを確認し、
// | 　　        受信していたら共用体r_udp_meridimに値を格納する
// | 引数　　　:  なし
// +----------------------------------------------------------------------
void receiveUDP()
{
  if (udp.parsePacket() >= MSG_BUFF) // データの受信バッファ確認
  {
    udp.read(r_udp_meridim.bval, MSG_BUFF); // データの受信
    udp_rsvd_flag = true;                   // 受信完了フラグを上げる
  }
  // delay(1);
}

// +----------------------------------------------------------------------
// | 関数名　　:  sendUDP()
// +----------------------------------------------------------------------
// | 機能     :  共用体s_udp_meridimをUDP通信で送信する
// | 引数　　　:  なし
// +----------------------------------------------------------------------
void sendUDP()
{
  udp.beginPacket(WIFI_SEND_IP, UDP_SEND_PORT); // UDPパケットの開始
  udp.write(s_udp_meridim.bval, MSG_BUFF);
  udp.endPacket(); // UDPパケットの終了
}

// +----------------------------------------------------------------------
// | 関数名　　:  joypad_read()
// +----------------------------------------------------------------------
// | 機能     :  リモコンの入力データを受け取り, PS2/3リモコン配列形式で返す
// | 引数１　　:  int. リモコンの種類(現在は2:KRC-5FHのみ)
// | 引数２　　:  uint64_t. 前回の受信値(8バイト分, 共用体データを想定)
// | 引数３　　:  bool. JOYPADの受信ボタンデータをこのデバイスで0リセットするか、
// | 　　　　　　　リセットせず論理加算するか （0:overide, 1:reflesh, 通常は1）
// | 戻値　　　:  uint64_t. リモコン受信値を格納
// +----------------------------------------------------------------------
uint64_t joypad_read(int mount_joypad, uint64_t pre_val, int polling, bool joypad_reflesh)
{
  if (mount_joypad == 2)
  { // KRR5FH(KRC-5FH)をICS_R系に接続している場合
    joypad_frame_count++;
    if (joypad_frame_count >= polling)
    {
      unsigned short buttonData;
      unsigned short pad_btn_tmp = 0;

      buttonData = krs_R.getKrrButton();
      delayMicroseconds(2);
      if (buttonData != KRR_BUTTON_FALSE) // ボタンデータが受信できていたら
      {
        int button_1 = buttonData;

        if ((button_1 & 15) == 15)
        { // 左側十字ボタン全部押しなら start押下とみなす
          pad_btn_tmp += 1;
        }
        else
        {
          // 左側の十字ボタン
          pad_btn_tmp += (button_1 & 1) * 16 + ((button_1 & 2) >> 1) * 64 + ((button_1 & 4) >> 2) * 32 + ((button_1 & 8) >> 3) * 128;
        }
        if ((button_1 & 368) == 368)
          pad_btn_tmp += 8; // 右側十字ボタン全部押しなら select押下とみなす
        else
        {
          // 右側十字ボタン
          pad_btn_tmp += ((button_1 & 16) >> 4) * 4096 + ((button_1 & 32) >> 5) * 16384 + ((button_1 & 64) >> 6) * 8192 + ((button_1 & 256) >> 8) * 32768;
        }
        // L1,L2,R1,R2
        pad_btn_tmp += ((button_1 & 2048) >> 11) * 2048 + ((button_1 & 4096) >> 12) * 512 + ((button_1 & 512) >> 9) * 1024 + ((button_1 & 1024) >> 10) * 256;
      }
      /* 共用体用の64ビットの上位16ビット部をボタンデータとして書き換える */
      uint64_t updated_val;
      if (joypad_reflesh)
      {
        updated_val = (pre_val & 0xFFFFFFFFFFFF0000) | (static_cast<uint64_t>(pad_btn_tmp)); // 上位16ビット index[0]
      }
      else
      {
        updated_val = (pre_val) | (static_cast<uint64_t>(pad_btn_tmp));
      }
      // updated_val = (updated_val & 0x0000FFFFFFFFFFFF) | (static_cast<uint64_t>(pad_btn_tmp) << 48); // 下位16ビット index[3]
      // updated_val = (updated_val & 0xFFFF0000FFFFFFFF) | (static_cast<uint64_t>(pad_btn_tmp) << 32); // 上位33-48ビット index[2]
      // updated_val = (updated_val & 0xFFFFFFFF0000FFFF) | (static_cast<uint64_t>(pad_btn_tmp) << 16); // 上位17-32ビット index[1]
      joypad_frame_count = 0;
      return updated_val;
    }
    else
    {
      return pre_val;
    }
  }
  else
  {
    return pre_val;
  }
}

// +----------------------------------------------------------------------
// | 関数名　　:  monitor_joypad(ushort *arr)
// +----------------------------------------------------------------------
// | 機能     :  リモコンの入力データを表示する.
// | 引数　　　:  ushort配列 4個.
// +----------------------------------------------------------------------
void monitor_joypad(ushort *arr)
{
  for (int i = 0; i < 4; i++)
  {
    Serial.print(arr[i]);
    if (i < 3)
    {
      Serial.print("/");
    }
  }
  Serial.println();
}

// +----------------------------------------------------------------------
// | 関数名　　:  check_sd()
// +----------------------------------------------------------------------
// | 機能     :  SDカードの初期化と読み書きテスト
// +----------------------------------------------------------------------
void check_sd()
{
  if (MOUNT_SD)
  {
    Serial.print("SD card check...");
    delay(100);
    if (!SD.begin(PIN_CHIPSELECT_SD))
    {
      Serial.println(" initialization FALIED!");
      delay(500);
    }
    else
    {
      Serial.println(" OK.");
    }

    if (CHECK_SD_RW)
    {
      // open the file.
      myFile = SD.open("/tmp.txt", FILE_WRITE);
      delayMicroseconds(2); // SPI安定化検証用

      // if the file opened okay, write to it:
      if (myFile)
      {
        Serial.print("SD card r/w check...");
        // SD書き込みテスト用のランダムな4桁の数字を生成
        randomSeed(long(analogRead(A0))); // 未接続ピンのノイズを利用
        int randNumber = random(1000, 9999);

        Serial.print(" write code ");
        Serial.print(randNumber);
        myFile.println(randNumber);
        delayMicroseconds(2); // SPI安定化検証用
        // close the file:
        myFile.close();
        Serial.print(" and");
        delayMicroseconds(10); // SPI安定化検証用
        // re-open the file for reading:
        myFile = SD.open("/tmp.txt");
        if (myFile)
        {
          Serial.print(" read code ");
          while (myFile.available())
          {
            Serial.write(myFile.read());
          }
          // close the file:
          myFile.close();
        }
        SD.remove("/tmp.txt");
        delay(10);
      }
      else
      {
        // if the file didn't open, print an error:
        Serial.println("Could not open SD test file.");
      }
    }
  }
  else
  {
    Serial.println("No SD reader/writer mounted.");
  }
}

// +----------------------------------------------------------------------
// | 関数名　　:  init_wifi()
// +----------------------------------------------------------------------
// | 機能     :  wifiの初期化
// +----------------------------------------------------------------------
void init_wifi(const char *wifi_ap_ssid, const char *wifi_ap_pass)
{
  WiFi.disconnect(true, true); // WiFi接続をリセット
  delay(100);
  WiFi.begin(wifi_ap_ssid, wifi_ap_pass); // Wifiに接続
  while (WiFi.status() != WL_CONNECTED)
  {            // https://www.arduino.cc/en/Reference/WiFiStatus 戻り値一覧
    delay(50); // 接続が完了するまでループで待つ
  }
}

// +----------------------------------------------------------------------
// | 関数名　　:  init_imuahrs()
// +----------------------------------------------------------------------
// | 機能     :  センサーの初期化(BNO055のみ)
// +----------------------------------------------------------------------
void init_imuahrs(int mount_imuahrs)
{
  if (mount_imuahrs == 3)
    if (!bno.begin())
    {
      Serial.println("No BNO055 detected ... Check your wiring or I2C ADDR!");
    }
    else
    {
      Serial.println("BNO055 mounted and started.");
      delay(50);
      bno.setExtCrystalUse(false);
      delay(10);
    }
  else
  {
    Serial.println("No IMU/AHRS sensor mounted.");
  }
}

// +----------------------------------------------------------------------
// | スレッド用関数　:  Core1_bno055_r(void *args)
// +----------------------------------------------------------------------
// | 機能    　　　 :  bno055のデータを取得し、bno055_read[]に書き込む
// +----------------------------------------------------------------------
void Core1_bno055_r(void *args)
{
  while (1)
  {
    /* 加速度センサ値の取得と表示 - VECTOR_ACCELEROMETER - m/s^2 */
    imu::Vector<3> accelermetor = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    bno055_read[0] = (float)accelermetor.x();
    bno055_read[1] = (float)accelermetor.y();
    bno055_read[2] = (float)accelermetor.z();

    /* ジャイロセンサ値の取得 - VECTOR_GYROSCOPE - rad/s */
    imu::Vector<3> gyroscope = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    bno055_read[3] = gyroscope.x();
    bno055_read[4] = gyroscope.y();
    bno055_read[5] = gyroscope.z();

    /* 磁力センサ値の取得と表示  - VECTOR_MAGNETOMETER - uT */
    imu::Vector<3> magnetmetor = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
    bno055_read[6] = magnetmetor.x();
    bno055_read[7] = magnetmetor.y();
    bno055_read[8] = magnetmetor.z();

    /* センサフュージョンによる方向推定値の取得と表示 - VECTOR_EULER - degrees */
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    bno055_read[12] = euler.y();                    // DMP_ROLL推定値
    bno055_read[13] = euler.z();                    // DMP_PITCH推定値
    bno055_read[14] = euler.x() - yaw_center - 180; // DMP_YAW推定値

    /*
      // センサフュージョンの方向推定値のクオータニオン
      imu::Quaternion quat = bno.getQuat();
      Serial.print("qW: "); Serial.print(quat.w(), 4);
      Serial.print(" qX: "); Serial.print(quat.x(), 4);
      Serial.print(" qY: "); Serial.print(quat.y(), 4);
      Serial.print(" qZ: "); Serial.println(quat.z(), 4);
    */

    /*
    // キャリブレーションのステータスの取得と表示
    uint8_t system, gyro, accel, mag = 0;
    bno.getCalibration(&system, &gyro, &accel, &mag);
    Serial.print("CALIB Sys:"); Serial.print(system, DEC);
    Serial.print(", Gy"); Serial.print(gyro, DEC);
    Serial.print(", Ac"); Serial.print(accel, DEC);
    Serial.print(", Mg"); Serial.println(mag, DEC);
    */
    delay(IMUAHRS_POLLING);
  }
}
