// Meridan Board -LITE-
// ESP32用スケッチ　20220614版
// Meridian_LITE_20220614 By Izumi Ninagawa & Meridian Project
// MIT Licenced.

#define VERSION "Hello, This is Meridian_LITE_20220614." // バージョン表示

// ESP32devkitC を1枚搭載するMeridan Board -LITE-用のコード
// 100Hz通信で安定動作。
// 現在使える構成要素は
// ① ICSサーボL,R 11個ずつ合計22軸分。
// ② リモコン KRC-5FH + KRR-5FH
// ③ 9軸センサ BNO005
// ④ SDカード(SPIに接続)

// ESP32 DevkitC + PlatformIOで使うことを想定
// PlatformIO上でのESP32のボードバージョンは3.5.0が良い。（4系はうまく動かないらしい）
// Serial1を使うには設定を変更する必要あり. 参考 → https://qiita.com/Ninagawa_Izumi/items/8ce2d55728fd5973087d

// サーボ値の返信は取りこぼしなどあるが、取りこぼす直前のデータを使うことで擬似的に安定させている。
// そのため返信値は参考値であり、基本的にはサーボ値を指示して動作させることが基本的な利用法となる。

//================================================================================================================
//---- E S P 3 2 の 配 線  ----------------------------------------------------------------------------------------
//================================================================================================================
/*
 ESP32devkitC  -  デバイス
   3V3         -  BNO005 VIN
   21          -  BNO005 SCL
   22          -  BNO005 SDA
   GND         -  BNO005 GND

   4  EN       -  ICS変換基板 R系統 EN
   16 RX       -  ICS変換基板 R系統 TX
   17 TX       -  ICS変換基板 R系統 RX
   5V          -  ICS変換基板 IOREF
   GND         -  ICS変換基板 GND

   33 EN       -  ICS変換基板 L系統 EN
   32 RX       -  ICS変換基板 L系統 TX
   27 TX       -  ICS変換基板 L系統 RX
   5V          -  ICS変換基板 IOREF
   GND         -  ICS変換基板 GND
*/

//================================================================================================================
//---- 初 期 設 定  -----------------------------------------------------------------------------------------------
//================================================================================================================

/* ライブラリ導入 */
#include <Arduino.h>
#include <WiFi.h>               // WiFi通信用ライブラリ
#include <WiFiUdp.h>            // UDP通信用ライブラリ
#include <IcsHardSerialClass.h> // KONDOサーボのライブラリ
#include <Wire.h>               // I2C通信用ライブラリ
#include <Adafruit_BNO055.h>    // 9軸センサBNO055用のライブラリ
#include <SPI.h>                // SPI自体は使用しないがBNO055に必要
#include <SD.h>                 //SDカード用のライブラリ導入

/* 頻繁に変更するであろう変数 #DEFINE */
#define FRAME_DURATION 10               // 1フレームあたりの単位時間（単位ms）
#define AP_SSID "xxxxxx"             // アクセスポイントのAP_SSID
#define AP_PASS "xxxxxx"              // アクセスポイントのパスワード
#define SEND_IP "192.168.1.xx"          // 送り先のPCのIPアドレス（PCのIPアドレスを調べておく）
#define BT_MAC_ADDR "xx:xx:xx:xx:xx:xx" // ESP32自身のBluetoothMACアドレス（本プログラムを実行しシリアルモニタで確認）
#define SD_MOUNT 1                      // SDリーダーの搭載 (0:なし, 1:あり)
#define IMU_MOUNT 1                     // 6軸or9軸センサーの搭載 (0:なし, 1:BNO055, 2:MPU6050(未実装))
#define JOYPAD_MOUNT 2                  // ジョイパッドの搭載 (現在2のKRC-5FHのみ有効, ジョイパッドを接続しない場合は0)
                                        // 0:なし、1:SBDBT(実), 2:KRC-5FH, 3:PS3(実), 4:PS4(実),5:Wii_yoko(実), 6:Wii+Nun(実), 7:WiiPRO(実), 8:Xbox(実)
#define JOYPAD_POLLING 4                // ジョイパッドの問い合わせフレーム間隔(推奨値:KRC-5FHは4,PSは10)
#define IMUAHRS_POLLING 10              // IMU,AHRSの問い合わせフレーム間隔
bool UDP_SEND = true;                   // PCへのデータ送信を行うか
bool UDP_RESEIVE = true;                // PCからのデータ受信を行うか

/* マスターコマンド定義 */
#define SET_YAW_CENTER 1002  // センサの推定ヨー軸を現在値センターとしてリセット
#define ENTER_TRIM_MODE 1003 // トリムモードに入る（全サーボオンで垂直に気おつけ姿勢で立つ）

/* 各種設定 #DEFINE */
#define MSG_SIZE 90       // Meridim配列の長さ設定（デフォルトは90）
#define SEND_PORT 22222   // 送り先のポート番号
#define RESV_PORT 22224   // このESP32のポート番号
#define ERR_LED 25        // LED用 処理が時間内に収まっていない場合に点灯
#define BAUDRATE1 1250000 // サーボの通信速度
#define TIMEOUT1 2        // サーボ返信エラーをスルーするのに程よい設定が2
#define EN_PIN_L 33       // サーボL系統のENピン
#define EN_PIN_R 4        // サーボR系統のENピン
#define SERVO_NUM_L 11    // L系統につないだサーボの数
#define SERVO_NUM_R 11    // R系統につないだサーボの数
#define CHIPSELECT_SD 15  // SDカード用のCSピン

IcsHardSerialClass krs_L(&Serial1, EN_PIN_L, BAUDRATE1, TIMEOUT1); //サーボL系統UARTの設定（TX27,RX32,EN33）
IcsHardSerialClass krs_R(&Serial2, EN_PIN_R, BAUDRATE1, TIMEOUT1); //サーボR系統UARTの設定（TX17,RX16,EN4）
const int MSG_BUFF = MSG_SIZE * 2;                                 // Meridim配列の長さ設定（デフォルトは90）

/* マルチスレッド用変数 */
TaskHandle_t thp[4]; //マルチスレッドのタスクハンドル格納用

/* リモコン用変数 */
unsigned short pad_btn = 0;
short pad_stick_R_x = 0;
short pad_stick_R_y = 0;
short pad_stick_L_x = 0;
short pad_stick_L_y = 0;
short pad_R2_val = 0;
short pad_L2_val = 0;
long pad_btn_disp = 65536;

/* センサー用変数 */
float bno055_read[16]; // bno055のデータの一時保存
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
float yaw_center = 0;

/* Meridim配列用の共用体の設定 */
typedef union
{
  short sval[MSG_SIZE];
  uint8_t bval[MSG_BUFF];
} UnionData;
UnionData s_udp_meridim;   // UDP送信用共用体のインスタンスを宣言
UnionData r_udp_meridim;   // UDP受信用共用体のインスタンスを宣言
UnionData pad_udp_meridim; // UDP送信用共用体のインスタンスを宣言

/* 変数一般 */
long frame_count = 0;
long frame_ms = FRAME_DURATION;    // 1フレームあたりの単位時間(ms)
long merc = (long)millis();        // フレーム管理時計の時刻 Meridian Clock.
long curr_millis = (long)millis(); // 現在時刻を取得
long curr_micro = (long)micros();  // 現在時刻を取得
char frame_sync_s = 0;             //フレーム毎に0-199をカウントし、送信用Meridm[88]の下位8ビットに格納
char frame_sync_r_expect = 0;      //フレーム毎に0-199をカウントし、受信値と比較
char frame_sync_r_resv = 0;        //今フレームに受信したframe_sync_rを格納
long error_count_udp = 0;
int frame_count_diff = 5;
int kr; // KRS角度データ一時保存用
int kl; // KRS角度データ一時保存用

float n;                    //計算用
int joypad_frame_count;     //ジョイパッドのポーリングカウント
bool monitor_joypad = true; //ジョイパッドのモニタリング用（後で消す）

/* フラグ用変数 */
bool udp_rsvd = 0; // UDPの受信終了フラグ

/* wifi設定 */
WiFiUDP udp;

// SDカードテスト用
File myFile;

/* KRSサーボのポジション用配列 */
int s_servo_pos_L[15]; // *備考:degreeではなくサーボ値が入る
int s_servo_pos_R[15];
int r_servo_pos_L[15];
int r_servo_pos_R[15];

/* 各サーボのマウントありなし */
bool idl_mt[15]; // L系統　*備考:mtはmountの略
bool idr_mt[15]; // R系統

/* 各サーボの回転方向順逆補正用 */
int idl_cw[15]; // L系統　*備考:cwはclockwise rotationの略
int idr_cw[15]; // R系統　 サーボ設定に対して順回転するかどうか(+1,-1)

/* 各サーボの直立ポーズトリム値 */
int idl_trim[15]; // L系統
int idr_trim[15]; // R系統

/* 各サーボのポジション値 */
int idl_mov[15]; // L系統
int idr_mov[15]; // R系統

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
// | 関数名　　:  float2HfShort(float val)
// +----------------------------------------------------------------------
// | 機能　　　:  floatを100倍し小数点以下を四捨五入した整数をshortに収める
// | 引数　　　:  float型. 有効数字　-327.67 ~ 327.67
// | 戻り値　　:  short型. 限界値を超えたら限界値を返す(-32767,32767)
// +----------------------------------------------------------------------
short float2HfShort(float val)
{
  int x = round(val * 100); // floatの小数点以下を四捨五入して整数化
  if (x > 32766)
  {
    x = 32767;
  }
  else if (x < -32766)
  {
    x = -32767;
  }
  return (short)x;
}

// +----------------------------------------------------------------------
// | 関数名　　:  float2HfInt(float val)
// +----------------------------------------------------------------------
// | 機能　　　:  floatを100倍し小数点以下を四捨五入した整数をintに収める
// | 引数　　　:  float型. 有効数字8桁(例:123.45678, -12345.678)
// | 戻り値　　:  int型.
// +----------------------------------------------------------------------
int float2HfInt(float val)
{
  int x = round(val * 100); // 100倍し小数点以下を四捨五入
  return x;
}

// +----------------------------------------------------------------------
// | 関数名　　:  Krs2HfDeg(int krs, int n, int pn)
// +----------------------------------------------------------------------
// | 機能     :  KRS単位値をdegree値*100に変換. HfはHundredfoldの略.
// | 引数１　　:  int型. サーボ値（KRS単位値）
// | 引数２　　:  int型. トリム値（KRS単位値）       具体例：idl_trim[i], idr_trim[i]
// | 引数３　　:  int型. 回転方向の順逆補正（+1,-1）　具体例：idl_cw[i],idr_cw[i]
// | 戻り値　　:  int型. degree値*100
// +----------------------------------------------------------------------
int Krs2HfDeg(int krs, int n, int pn)
{
  float x = (krs - 7500 - n) * 3.375 * pn;
  return int(x);
}

// +----------------------------------------------------------------------
// | 関数名　　:  HfDeg2Krs(int hfdegree, int n, int pn)
// +----------------------------------------------------------------------
// | 機能     :  degree値*100 をKRS単位値に変換. HfはHundredfoldの略.
// | 引数１　　:  float型. degree値
// | 引数２　　:  int型. トリム値（KRS単位値）       具体例：idl_trim[i], idr_trim[i]
// | 引数３　　:  int型. 回転方向の順逆補正（+1,-1）　具体例：idl_cw[i],idr_cw[i]
// | 戻り値　　:  int型. KRS単位値（3500-11500）
// | 備考　　　:  0.02度ぐらいからサーボ値には反映される(=0.59で1に繰り上がる)
// +----------------------------------------------------------------------
int HfDeg2Krs(int hfdegree, int n, int pn)
{
  float x = 7500 + n + (hfdegree / 3.375 * pn); //
  if (x > 11500)                                //上限を設定
  {
    x = 11500;
  }
  else if (x < 3500) //下限を設定
  {
    x = 3500;
  }
  return int(x);
}

// +----------------------------------------------------------------------
// | 関数名　　:  checksum_val(short arr[], int len)
// +----------------------------------------------------------------------
// | 機能     :  配列のチェックサムを算出
// | 　　     :  チェックサムは配列の末尾を除く合計数をビット反転しShort型にしたもの
// | 引数１　　:  Meridim配列(Short型の配列)
// | 引数２　　:  配列の長さ
// | 戻り値　　:  short型. チェックサム値
// +----------------------------------------------------------------------
short checksum_val(short arr[], int len)
{
  int cksm = 0;
  for (int i = 0; i < len - 1; i++)
  {
    cksm += int(arr[i]);
  }
  return short(~cksm);
}

// +-------------------------------------------------------------------
// | 関数名　　:  checksum_rslt(short arr[], int len)
// +-------------------------------------------------------------------
// | 機能     :  チェックサムの合否判定
// | 　　     :  配列の末尾を除く合計数をビット反転しShort型にしたものと、末尾の値を比較
// | 引数１　　:  Meridim配列(Short型の配列)
// | 引数２　　:  配列の長さ
// | 戻り値　　:  bool型. チェックサム値がOKならtrue, NGならfalse
// +------------------------------------------------------------------
bool checksum_rslt(short arr[], int len)
{
  int cksm = 0;
  for (int i = 0; i < len - 1; i++)
  {
    cksm += int(arr[i]);
  }
  if (short(~cksm) == arr[len - 1])
  {
    return true;
  }
  return false;
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
  if (udp.parsePacket() >= MSG_BUFF) //データの受信バッファ確認
  {
    udp.read(r_udp_meridim.bval, MSG_BUFF); //データの受信
    udp_rsvd = true;                        //受信完了フラグを上げる
  }
  // delay(1);
}

// +----------------------------------------------------------------------
// | 関数名　　:  receiveUDP()
// +----------------------------------------------------------------------
// | 機能     :  共用体s_udp_meridimをUDP通信で送信する
// | 引数　　　:  なし
// +----------------------------------------------------------------------
void sendUDP()
{
  udp.beginPacket(SEND_IP, SEND_PORT); // UDPパケットの開始
  udp.write(s_udp_meridim.bval, MSG_BUFF);
  udp.endPacket(); // UDPパケットの終了
}

// +----------------------------------------------------------------------
// | 関数名　　:  joypad_read()
// +----------------------------------------------------------------------
// | 機能     :  KRC-5FHのデータをKRR-FH経由で受け取りpad_btnに転記する
// | 引数　　　:  なし
// +----------------------------------------------------------------------
// ■ JOYPAD処理 ---------------------------------------------------------------
void joypad_read()
{
  if (JOYPAD_MOUNT == 2)
  { // KRR5FH(KRC-5FH)をICS_R系に接続している場合
    joypad_frame_count++;
    if (joypad_frame_count >= JOYPAD_POLLING)
    {
      unsigned short buttonData;
      buttonData = krs_R.getKrrButton();
      delayMicroseconds(2);
      if (buttonData != KRR_BUTTON_FALSE) //ボタンデータが受信できていたら
      {
        int button_1 = buttonData;
        // if (monitor_joypad) {

        pad_btn = 0;
        if ((button_1 & 15) == 15)
        { //左側十時ボタン全部押しならstart押下とみなす
          pad_btn += 1;
        }
        else
        {
          //左側の十時ボタン
          pad_btn += (button_1 & 1) * 16 + ((button_1 & 2) >> 1) * 64 + ((button_1 & 4) >> 2) * 32 + ((button_1 & 8) >> 3) * 128;
        }
        if ((button_1 & 368) == 368)
          pad_btn += 8; //右側十時ボタン全部押しならselect押下とみなす
        else
        {
          //右側十時ボタン
          pad_btn += ((button_1 & 16) >> 4) * 4096 + ((button_1 & 32) >> 5) * 16384 + ((button_1 & 64) >> 6) * 8192 + ((button_1 & 256) >> 8) * 32768;
        }
        // L1,L2,R1,R2
        pad_btn += ((button_1 & 2048) >> 11) * 2048 + ((button_1 & 4096) >> 12) * 512 + ((button_1 & 512) >> 9) * 1024 + ((button_1 & 1024) >> 10) * 256;
        //}
      }
      joypad_frame_count = 0;
    }
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
    // 加速度センサ値の取得と表示 - VECTOR_ACCELEROMETER - m/s^2
    imu::Vector<3> accelermetor = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    bno055_read[0] = (float)accelermetor.x();
    bno055_read[1] = (float)accelermetor.y();
    bno055_read[2] = (float)accelermetor.z();

    // ジャイロセンサ値の取得 - VECTOR_GYROSCOPE - rad/s
    imu::Vector<3> gyroscope = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    bno055_read[3] = gyroscope.x();
    bno055_read[4] = gyroscope.y();
    bno055_read[5] = gyroscope.z();

    // 磁力センサ値の取得と表示  - VECTOR_MAGNETOMETER - uT
    imu::Vector<3> magnetmetor = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
    bno055_read[6] = magnetmetor.x();
    bno055_read[7] = magnetmetor.y();
    bno055_read[8] = magnetmetor.z();

    // センサフュージョンによる方向推定値の取得と表示 - VECTOR_EULER - degrees
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

//================================================================================================================
//---- S E T  U P -----------------------------------------------------------------------------------------------
//================================================================================================================
void setup()
{
  /* ピンモードの設定 */
  pinMode(ERR_LED, OUTPUT); // エラー用LED

  /* PC用シリアルの設定 */
  Serial.begin(500000);

  /* サーボモーター用シリアルの設定 */
  krs_L.begin();
  krs_R.begin();

  delay(200);
  Serial.println();
  Serial.println("Serial Start...");
  Serial.println();
  Serial.println(VERSION);

  Wire.begin(22, 21);

  /* センサの初期化 */
  if (IMU_MOUNT == 1)
  {
    if (!bno.begin())
    {
      Serial.println("No BNO055 detected ... Check your wiring or I2C ADDR!");
      // while (1)
      //   ;
    }
    else
    {
      Serial.println("BNO055 mounted.");
      delay(50);
      bno.setExtCrystalUse(false);
      delay(10);
    }
  }
  else
  {
    Serial.println("No IMU/AHRS sensor mounted.");
  }

  /* スレッドの開始 */
  // センサー用スレッド
  xTaskCreatePinnedToCore(Core1_bno055_r, "Core1_bno055_r", 4096, NULL, 10, &thp[0], 1);
  delay(10);

  // SDカードの初期設定と書き込み
  if (SD_MOUNT)
  {
    Serial.print("Initializing SD card...");

    if (!SD.begin(CHIPSELECT_SD))
    {
      Serial.println(" initialization FALIED!");
      delay(500);
      // Serial.println("Retry.");
      // return;
    }
    else
    {
      Serial.println(" OK.");
    }

    // open the file.
    myFile = SD.open("/test.txt", FILE_WRITE);
    delayMicroseconds(1); // SPI安定化検証用

    // if the file opened okay, write to it:
    if (myFile)
    {
      Serial.print("SD card check...");
      // myFile.println("Meridian Board read write test.");
      randomSeed(analogRead(34) * 10 + analogRead(35) * 2 + analogRead(36));
      int randNumber = random(1000, 9999); // 0から299の乱数を生成
      Serial.print(" Writing code ");
      Serial.print(randNumber);
      // myFile.println(" Writing code ");
      myFile.println(randNumber);
      delayMicroseconds(1); // SPI安定化検証用
      // close the file:
      myFile.close();
      Serial.print(" and");
    }
    else
    {
      // if the file didn't open, print an error:
      Serial.println("... opening /test.txt FALIED!");
    }
    delayMicroseconds(1); // SPI安定化検証用
    // re-open the file for reading:
    myFile = SD.open("/test.txt");
    if (myFile)
    {
      // Serial.println("SD opening file /test.txt...");

      // read from the file until there's nothing else in it:
      // Serial.println("SD reading texts in test.txt:");
      Serial.print(" read code ");

      while (myFile.available())
      {
        Serial.write(myFile.read());
      }
      // close the file:
      myFile.close();
    }
    else
    {
      // if the file didn't open, print an error:
      Serial.println("... opening /test.txt FALIED!");
    }
    delay(100);
  }
  else
  {
    Serial.println("No SD reader mounted.");
  }

  if (JOYPAD_MOUNT == 2)
  {
    Serial.println("KRR-5FH(KRC-5FH) mounted.");
  }
  else
  {
    Serial.println("No Remort Controller mounted.");
  }

  /* WiFiの初期化と開始 */
  WiFi.disconnect(true, true); // WiFi接続をリセット
  // Serial.println("Connecting to WiFi to : " + String(AP_SSID)); //接続先を表示
  delay(100);
  WiFi.begin(AP_SSID, AP_PASS); // Wifiに接続
  while (WiFi.status() != WL_CONNECTED)
  {            // https://www.arduino.cc/en/Reference/WiFiStatus 返り値一覧
    delay(50); //接続が完了するまでループで待つ
  }
  Serial.println("WiFi connected to  => " + String(AP_SSID)); // WiFi接続完了通知

  Serial.print("PC's IP address is  => ");
  Serial.println(SEND_IP); // 送信先PCのIPアドレスの表示
  
  Serial.print("ESP32's IP address is  => ");
  Serial.println(WiFi.localIP()); // ESP32自身のIPアドレスの表示
  udp.begin(RESV_PORT);           // UDP通信の開始

  /* Bluetoothの初期化 */
  uint8_t bt_mac[6];
  String self_mac_address = "";
  esp_read_mac(bt_mac, ESP_MAC_BT); // ESP32自身のBluetoothMacアドレスを表示
  self_mac_address = String(bt_mac[0], HEX) + ":" + String(bt_mac[1], HEX) + ":" + String(bt_mac[2], HEX) + ":" + String(bt_mac[3], HEX) + ":" + String(bt_mac[4], HEX) + ":" + String(bt_mac[5], HEX);
  Serial.print("ESP32's Bluetooth Mac Address is => " + self_mac_address);
  Serial.println();
  delay(1000);
  delay(1000);

  /* 各サーボのマウントありなし（1:サーボあり、0:サーボなし） */
  idl_mt[0] = true;   //頭ヨー
  idl_mt[1] = true;   //左肩ピッチ
  idl_mt[2] = true;   //左肩ロール
  idl_mt[3] = true;   //左肘ヨー
  idl_mt[4] = true;   //左肘ピッチ
  idl_mt[5] = true;   //左股ヨー
  idl_mt[6] = true;   //左股ロール
  idl_mt[7] = true;   //左股ピッチ
  idl_mt[8] = true;   //左膝ピッチ
  idl_mt[9] = true;   //左足首ピッチ
  idl_mt[10] = true;  //左足首ロール
  idl_mt[11] = false; //予備
  idl_mt[12] = false; //予備
  idl_mt[13] = false; //予備
  idl_mt[14] = false; //予備
  idr_mt[0] = true;   //腰ヨー
  idr_mt[1] = true;   //右肩ピッチ
  idr_mt[2] = true;   //右肩ロール
  idr_mt[3] = true;   //右肘ヨー
  idr_mt[4] = true;   //右肘ピッチ
  idr_mt[5] = true;   //右股ヨー
  idr_mt[6] = true;   //右股ロール
  idr_mt[7] = true;   //右股ピッチ
  idr_mt[8] = true;   //右膝ピッチ
  idr_mt[9] = true;   //右足首ピッチ
  idr_mt[10] = true;  //右足首ロール
  idr_mt[11] = false; //予備
  idr_mt[12] = false; //予備
  idr_mt[13] = false; //予備
  idr_mt[14] = false; //予備

  /* 各サーボの内外回転プラマイ方向補正(1 or -1) */
  idl_cw[0] = 1;  //頭ヨー
  idl_cw[1] = 1;  //左肩ピッチ
  idl_cw[2] = 1;  //左肩ロール
  idl_cw[3] = 1;  //左肘ヨー
  idl_cw[4] = 1;  //左肘ピッチ
  idl_cw[5] = 1;  //左股ヨー
  idl_cw[6] = 1;  //左股ロール
  idl_cw[7] = 1;  //左股ピッチ
  idl_cw[8] = 1;  //左膝ピッチ
  idl_cw[9] = 1;  //左足首ピッチ
  idl_cw[10] = 1; //左足首ロール
  idl_cw[11] = 1; //予備
  idl_cw[12] = 1; //予備
  idl_cw[13] = 1; //予備
  idl_cw[14] = 1; //予備
  idr_cw[0] = 1;  //腰ヨー
  idr_cw[1] = 1;  //右肩ピッチ
  idr_cw[2] = 1;  //右肩ロール
  idr_cw[3] = 1;  //右肘ヨー
  idr_cw[4] = 1;  //右肘ピッチ
  idr_cw[5] = 1;  //右股ヨー
  idr_cw[6] = 1;  //右股ロール
  idr_cw[7] = 1;  //右股ピッチ
  idr_cw[8] = 1;  //右膝ピッチ
  idr_cw[9] = 1;  //右足首ピッチ
  idr_cw[10] = 1; //右足首ロール
  idr_cw[11] = 1; //予備
  idr_cw[12] = 1; //予備
  idr_cw[13] = 1; //予備
  idr_cw[14] = 1; //予備

  /* 各サーボの直立デフォルト値　(KRS値  0deg=7500, +-90deg=7500+-2667  KRS値=deg/0.03375) */
  idl_trim[0] = 0;     //頭ヨー        /* 直立状態になるよう、具体的な数値を入れて現物調整する */
  idl_trim[1] = -70;   //左肩ピッチ
  idl_trim[2] = -2700; //左肩ロール
  idl_trim[3] = 0;     //左肘ヨー
  idl_trim[4] = 2666;  //左肘ピッチ
  idl_trim[5] = 0;     //左股ヨー
  idl_trim[6] = 0;     //左股ロール
  idl_trim[7] = -40;   //左股ピッチ
  idl_trim[8] = -1720; //左膝ピッチ
  idl_trim[9] = -600;  //左足首ピッチ
  idl_trim[10] = -20;  //左足首ロール
  idl_trim[11] = 0;    //予備
  idl_trim[12] = 0;    //予備
  idl_trim[13] = 0;    //予備
  idl_trim[14] = 0;    //予備
  idr_trim[0] = 0;     //腰ヨー
  idr_trim[1] = 0;     //右肩ピッチ
  idr_trim[2] = -2650; //右肩ロール
  idr_trim[3] = 0;     //右肘ヨー
  idr_trim[4] = 2666;  //右肘ピッチ
  idr_trim[5] = 0;     //右股ヨー
  idr_trim[6] = 50;    //右股ロール
  idr_trim[7] = -100;  //右股ピッチ
  idr_trim[8] = -1700; //右膝ピッチ
  idr_trim[9] = -600;  //右足首ピッチ
  idr_trim[10] = -70;  //右足首ロール
  idr_trim[11] = 0;    //予備
  idr_trim[12] = 0;    //予備
  idr_trim[13] = 0;    //予備
  idr_trim[14] = 0;    //予備

  delay(100);

  merc += (long)millis() + 10; // mercをリセット
}

//================================================================================================================
//---- M A I N  L O O P -----------------------------------------------------------------------------------------
//================================================================================================================
void loop()
{

  //////// < 1 > U D P 送 信 信 号 作 成 ////////////////////////////////////////////

  // @ [1-1] センサーからの値を送信用に格納
  if (IMU_MOUNT == 1)
  {
    for (int i = 0; i < 15; i++)
    {
      s_udp_meridim.sval[i] = float2HfShort(bno055_read[i]); // DMP_ROLL推定値
    }
  }

  // @ [1-2] フレームスキップ検出用のカウントをカウントアップして送信用に格納
  frame_sync_s++;
  if (frame_sync_s > 199)
  {
    frame_sync_s = 0;
  }
  s_udp_meridim.bval[176] = frame_sync_s;

  // @ [1-3] チェックサムを計算して格納
  s_udp_meridim.sval[MSG_SIZE - 1] = checksum_val(s_udp_meridim.sval, MSG_SIZE);

  //////// < 2 > U D P 送 信 ///////////////////////////////////////////////////////

  // @ [2-1] UDP送信を実行
  if (UDP_SEND) //設定でUDPの送信を行うかどうか決定
  {
    sendUDP();
  }
  delayMicroseconds(5);

  //////// < 3 > U D P 受 信 ///////////////////////////////////////////////////////

  // @ [3-1] UDP受信の実行 もしデータパケットが来ていれば受信する
  if (UDP_RESEIVE) //設定でUDPの受信を行うかどうか決定
  {
    receiveUDP();
  }
  //　→ ここでr_udp_meridim.sval に受信したMeridim配列が入っている状態。

  // @ [3-2] UDP受信配列から UDP送信配列にデータを転写
  memcpy(s_udp_meridim.bval, r_udp_meridim.bval, MSG_SIZE * 2);

  //////// < 4 > リ モ コ ン 受 信 ///////////////////////////////////////////////////

  // @ [4-2] コントロールパッド受信値の転記
  if (JOYPAD_MOUNT == 1)
  { // SBDBTが接続設定されていれば受信チェック（未実装）
    Serial.print("SBDBT connection has not been programmed yet.");
  }
  if (JOYPAD_MOUNT == 2)
  { // KRC-5FH+KRR-5FHが接続設定されていれば受信チェック
    joypad_read();
    s_udp_meridim.sval[80] = pad_btn;
  }

  // @ [4-2] コントロールパッド受信値の転記
  if (JOYPAD_MOUNT == 4)
  {
    for (int i = 80; i < 84; i++)
    {
      s_udp_meridim.sval[i] = pad_udp_meridim.sval[i];
    }
  }

  //////// < 5 >  受 信 コ マ ン ド に 基 づ く 制 御 処 理 ////////////////////////////

  // @ [5-1] AHRSのヨー軸リセット
  if (r_udp_meridim.sval[0] == SET_YAW_CENTER)
  {
    setyawcenter();
  }

  //////// < 6 >  E S P 内 部 で 位 置 制 御 す る 場 合 の 処 理 //////////////////////

  /* 現在はとくに設定なし */
  // n = cos(PI / 2 / 300 * frame_count) * 500;

  //////// < 7 >  サ ー ボ 動 作 の 実 行 ////////////////////////////////////////////

  // @ [7-1] 前回のラストに読み込んだサーボ位置をサーボ配列に書き込む
  for (int i = 0; i < 15; i++)
  {
    s_servo_pos_L[i] = HfDeg2Krs(int(r_udp_meridim.sval[i * 2 + 21]), idl_trim[i], idl_cw[i]);
    s_servo_pos_R[i] = HfDeg2Krs(int(r_udp_meridim.sval[i * 2 + 51]), idr_trim[i], idr_cw[i]);
  }

  // @ [7-2] チェックサムを確認
  if (checksum_rslt(r_udp_meridim.sval, MSG_SIZE)) // Check sum OK!
  {
    s_udp_meridim.bval[177] &= B10111111; //エラーフラグ14番(ESP32のPCからのUDP受信エラー検出)をオフ

    // @ [7-3] サーボ受信値の処理
    for (int i = 0; i < SERVO_NUM_R; i++) //接続したサーボの数だけ繰り返す。最大は15?
    {
      if (idl_mt[i]) // L系統のサーボのマウントありなしを確認
      {
        if (r_udp_meridim.sval[(i * 2) + 20] == 1) //サーボコマンドが1ならサーボをオンして位置を指定
        {

          kl = krs_L.setPos(i, HfDeg2Krs(r_udp_meridim.sval[i * 2 + 21], idl_trim[i], idl_cw[i]));
          // delayMicroseconds(1); //安定化検証用
          if (kl == -1) // サーボからの返信信号を受け取れなかった時は前回の数値のままにする
          {
            kl = s_servo_pos_L[i];
          }
        }
        else //サーボコマンドが0ならサーボをオフして位置を取得
        {
          kl = krs_L.setFree(i); // 1以外ならとりあえずサーボを脱力し位置を取得。手持ちの最大は15
          if (kl == -1)          // サーボからの返信信号を受け取れなかった時は前回の数値のままにする
          {
            kl = s_servo_pos_L[i];
          }
        } //サーボの返り値を送信Meridim配列に格納
        s_udp_meridim.sval[(i * 2) + 21] = short(Krs2HfDeg(kl, idr_trim[i], idr_cw[i]));
      }
      // delayMicroseconds(2); //コツ. このディレイがあるとサーボの送受信がうまく動く.

      if (idr_mt[i]) // R系統のサーボのマウントありなしを確認
      {
        if (r_udp_meridim.sval[(i * 2) + 50] == 1) //サーボコマンドが1ならサーボをオンして位置を指定
        {
          kr = krs_R.setPos(i, HfDeg2Krs(r_udp_meridim.sval[i * 2 + 51], idr_trim[i], idr_cw[i]));
          if (kr == -1) // サーボからの返信信号を受け取れなかった時は前回の数値のままにする
          {
            kr = s_servo_pos_R[i];
          }
        }
        else //サーボコマンドが0ならサーボをオフして位置を取得
        {
          kr = krs_R.setFree(i);
          if (kr == -1) // サーボからの返信信号を受け取れなかった時は前回の数値のままにする
          {
            kr = s_servo_pos_R[i];
          }
        } //サーボの返り値を送信Meridim配列に格納
        s_udp_meridim.sval[(i * 2) + 51] = short(Krs2HfDeg(kr, idr_trim[i], idr_cw[i]));
      }
      delayMicroseconds(2); //コツ. このディレイがあるとサーボの送受信がうまく動く.
    }
  }
  else // Check sum NG*
  {
    error_count_udp++;
    s_udp_meridim.bval[177] |= B01000000; //エラーフラグ14番(ESP32のPCからのUDP受信エラー検出)をオン
  }

  //////// < 8 >  エ ラ ー リ ポ ー ト の 作 成 //////////////////////////////////////

  // @ [8-1] 通信エラー処理(スキップ検出)
  frame_sync_r_expect++; //フレームカウント予想値を加算
  if (frame_sync_r_expect > 199)
  { //予想値が200以上ならカウントを0に戻す
    frame_sync_r_expect = 0;
  }
  // frame_sync_rの確認(受信フレームにスキップが生じていないかをMeridim[88]の下位8ビットのカウンターで判断)
  frame_sync_r_resv = s_udp_meridim.bval[176]; //数値を受け取る

  if (frame_sync_r_resv == frame_sync_r_expect) // frame_sync_rの受信値が期待通りなら順番どおり受信
  {
    s_udp_meridim.bval[177] &= B11111011; //エラーフラグ10番(ESP受信のスキップ検出)をオフ
  }
  else
  {
    s_udp_meridim.bval[177] |= B00000100; //エラーフラグ10番(ESP受信のスキップ検出)をオン
    if (frame_sync_r_resv == frame_sync_r_expect - 1)
    {
      //同じ値を２回取得した場合には実際はシーケンスが進んだものとして補正（アルゴリズム要検討）
      frame_sync_r_expect = frame_sync_r_resv + 1;
    }
    else
    {
      frame_sync_r_expect = frame_sync_r_resv; //取りこぼしについては現在の受信値を正解の予測値としてキープ
    }
  }

  //////// < 9 >  フ レ ー ム 終 端 処 理 ///////////////////////////////////////////

  // @ [9-1] この時点で１フレーム内に処理が収まっていない時の処理
  curr_millis = (long)millis(); // 現在時刻を更新
  if (curr_millis > merc)
  {                                 // 現在時刻がフレーム管理時計を超えていたらアラートを出す
    Serial.println("[ERR] delay:"); //シリアルに遅延msを表示
    Serial.println(curr_millis - merc);
    digitalWrite(ERR_LED, HIGH); //処理落ちが発生していたらLEDを点灯
  }
  else
  {
    digitalWrite(ERR_LED, LOW); //処理が収まっていればLEDを消灯
  }

  // @ [9-2] この時点で時間が余っていたら時間消化。時間がオーバーしていたらこの処理を自然と飛ばす。
  curr_millis = (long)millis();
  curr_micro = (long)micros(); // 現在時刻を取得

  while ((merc - curr_millis) >= 1)
  {
    delay(1);
    curr_millis = (long)millis();
  }

  while (curr_micro < merc * 1000)
  {
    curr_micro = (long)micros(); // 現在時刻を取得
  }

  // @ [9-3] フレーム管理時計mercのカウントアップ
  merc = merc + frame_ms;                       //フレーム管理時計を1フレーム分進める
  frame_count = frame_count + frame_count_diff; //サインカーブ動作用のフレームカウントをいくつずつ進めるかをここで設定。
}
