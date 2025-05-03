#ifndef __MERIDIAN_MAIN_FUNC__
#define __MERIDIAN_MAIN_FUNC__

// ヘッダファイルの読み込み
#include "config.h"

// ライブラリ導入
#include <Adafruit_BNO055.h>            // 9軸センサBNO055用
#include <MPU6050_6Axis_MotionApps20.h> // MPU6050用
#include <Meridian.h>                   // Meridianのライブラリ導入
extern MERIDIANFLOW::Meridian mrd;
#include <IcsHardSerialClass.h> // ICSサーボのインスタンス設定
extern IcsHardSerialClass ics_L;
extern IcsHardSerialClass ics_R;

//------------------------------------------------------------------------------------
//  列挙型
//------------------------------------------------------------------------------------

enum UartLine { // サーボ系統の列挙型(L,R,C)
  L,            // Left
  R,            // Right
  C             // Center
};

enum ServoType { // サーボプロトコルのタイプ
  NOSERVO = 0,   // サーボなし
  PWM_S = 1,     // Single PWM (WIP)
  PCA9685 = 11,  // I2C_PCA9685 to PWM (WIP)
  FTBRSX = 21,   // FUTABA_RSxTTL (WIP)
  DXL1 = 31,     // DYNAMIXEL 1.0 (WIP)
  DXL2 = 32,     // DYNAMIXEL 2.0 (WIP)
  KOICS3 = 43,   // KONDO_ICS 3.5 / 3.6
  KOPMX = 44,    // KONDO_PMX (WIP)
  JRXBUS = 51,   // JRPROPO_XBUS (WIP)
  FTCSTS = 61,   // FEETECH_STS (WIP)
  FTCSCS = 62    // FEETECH_SCS (WIP)
};

enum ImuAhrsType { // 6軸9軸センサ種の列挙型(NO_IMU, MPU6050_IMU, MPU9250_IMU, BNO055_AHRS)
  NO_IMU = 0,      // IMU/AHRS なし.
  MPU6050_IMU = 1, // MPU6050
  MPU9250_IMU = 2, // MPU9250(未設定)
  BNO055_AHRS = 3  // BNO055
};

enum PadType {   // リモコン種の列挙型(NONE, PC, MERIMOTE, BLUERETRO, SBDBT, KRR5FH)
  NONE = 0,      // リモコンなし
  PC = 0,        // PCからのPD入力情報を使用
  MERIMOTE = 1,  // MERIMOTE(未導入)
  BLUERETRO = 2, // BLUERETRO(未導入)
  SBDBT = 3,     // SBDBT(未導入)
  KRR5FH = 4,    // KRR5FH
  WIIMOTE = 5,   // WIIMOTE / WIIMOTE + Nunchuk
  WIIMOTE_C = 6, // WIIMOTE+Classic
};

enum PadButton {  // リモコンボタンの列挙型
  PAD_SELECT = 1, // Select
  PAD_HOME = 2,   // HOME
  PAD_L3 = 2,     // L3
  PAD_R3 = 4,     // L4
  PAD_START = 8,  // Start
  PAD_UP = 16,    // 十字上
  PAD_RIGHT = 32, // 十字右
  PAD_DOWN = 64,  // 十字下
  PAD_LEFT = 128, // 十字左
  PAD_L2 = 256,   // L2
  PAD_R2 = 512,   // R2
  PAD_L1 = 1024,  // L1
  PAD_R1 = 2048,  // R1
  PAD_bU = 4096,  // △ 上
  PAD_bR = 8192,  // o 右
  PAD_bD = 16384, // x 下
  PAD_bL = 32768  // ◻︎ 左
};

enum BinHexDec { // 数値表示タイプの列挙型(Bin, Hex, Dec)
  Bin = 0,       // BIN
  Hex = 1,       // HEX
  Dec = 2,       // DEC
};

//------------------------------------------------------------------------------------
//  変数
//------------------------------------------------------------------------------------

// システム用の変数
const int MRDM_BYTE = MRDM_LEN * 2;    // Meridim配列のバイト型の長さ
const int MRD_ERR = MRDM_LEN - 2;      // エラーフラグの格納場所(配列の末尾から2つめ)
const int MRD_ERR_u = MRD_ERR * 2 + 1; // エラーフラグの格納場所(上位8ビット)
const int MRD_ERR_l = MRD_ERR * 2;     // エラーフラグの格納場所(下位8ビット)
const int MRD_CKSM = MRDM_LEN - 1;     // チェックサムの格納場所(配列の末尾)
const int PAD_LEN = 5;                 // リモコン用配列の長さ
TaskHandle_t thp[4];                   // マルチスレッドのタスクハンドル格納用

//------------------------------------------------------------------------------------
//  クラス・構造体・共用体
//------------------------------------------------------------------------------------

// Meridim配列用の共用体の設定
typedef union {
  short sval[MRDM_LEN + 4];           // short型で90個の配列データを持つ
  unsigned short usval[MRDM_LEN + 2]; // 上記のunsigned short型
  uint8_t bval[+4];                   // byte型で180個の配列データを持つ
  uint8_t ubval[MRDM_BYTE + 4];       // 上記のunsigned byte型
} Meridim90Union;
Meridim90Union s_udp_meridim;       // Meridim配列データ送信用(short型, センサや角度は100倍値)
Meridim90Union r_udp_meridim;       // Meridim配列データ受信用
Meridim90Union s_udp_meridim_dummy; // SPI送信ダミー用

// フラグ用変数
struct MrdFlags {
  bool imuahrs_available = true;        // メインセンサ値を読み取る間, サブスレッドによる書き込みを待機
  bool udp_board_passive = false;       // UDP通信の周期制御がボード主導(false) か, PC主導(true)か.
  bool count_frame_reset = false;       // フレーム管理時計をリセットする
  bool stop_board_during = false;       // ボードの末端処理をmeridim[2]秒, meridim[3]ミリ秒だけ止める.
  bool eeprom_write_mode = false;       // EEPROMへの書き込みモード.
  bool eeprom_read_mode = false;        // EEPROMからの読み込みモード.
  bool eeprom_protect = EEPROM_PROTECT; // EEPROMの書き込みプロテクト.
  bool eeprom_load = EEPROM_LOAD;       // 起動時にEEPROMの内容を読み込む
  bool eeprom_set = EEPROM_SET;         // 起動時にEEPROMに規定値をセット
  bool sdcard_write_mode = false;       // SDCARDへの書き込みモード.
  bool sdcard_read_mode = false;        // SDCARDからの読み込みモード.
  bool wire0_init = false;              // I2C 0系統の初期化合否
  bool wire1_init = false;              // I2C 1系統の初期化合否
  bool bt_busy = false;                 // Bluetoothの受信中フラグ(UDPコンフリクト回避用)
  bool spi_rcvd = true;                 // SPIのデータ受信判定
  bool udp_rcvd = false;                // UDPのデータ受信判定
  bool udp_busy = false;                // UDPスレッドでの受信中フラグ(送信抑制)

  bool udp_receive_mode = MODE_UDP_RECEIVE; // PCからのデータ受信実施(0:OFF, 1:ON, 通常は1)
  bool udp_send_mode = MODE_UDP_SEND;       // PCへのデータ送信実施(0:OFF, 1:ON, 通常は1)
  bool meridim_rcvd = false;                // Meridimが正しく受信できたか.
};
MrdFlags flg;

// シーケンス番号理用の変数
struct MrdSq {
  int s_increment = 0; // フレーム毎に0-59999をカウントし, 送信
  int r_expect = 0;    // フレーム毎に0-59999をカウントし, 受信値と比較
};
MrdSq mrdsq;

// タイマー管理用の変数
struct MrdTimer {
  long frame_ms = FRAME_DURATION; // 1フレームあたりの単位時間(ms)
  int count_loop = 0;             // サイン計算用の循環カウンタ
  int count_loop_dlt = 2;         // サイン計算用の循環カウンタを1フレームにいくつ進めるか
  int count_loop_max = 359999;    // 循環カウンタの最大値
  unsigned long count_frame = 0;  // メインフレームのカウント

  int pad_interval = (PAD_INTERVAL - 1 > 0) ? PAD_INTERVAL - 1 : 1; // パッドの問い合わせ待機時間
};
MrdTimer tmr;

// エラーカウント用
struct MrdErr {
  int esp_pc = 0;   // PCの受信エラー(ESP32からのUDP)
  int pc_esp = 0;   // ESP32の受信エラー(PCからのUDP)
  int esp_tsy = 0;  // Teensyの受信エラー(ESP32からのSPI)
  int tsy_esp = 0;  // ESP32の受信エラー(TeensyからのSPI)
  int esp_skip = 0; // UDP→ESP受信のカウントの連番スキップ回数
  int tsy_skip = 0; // ESP→Teensy受信のカウントの連番スキップ回数
  int pc_skip = 0;  // PC受信のカウントの連番スキップ回数
};
MrdErr err;

typedef union // リモコン値格納用
{
  short sval[PAD_LEN];        // short型で4個の配列データを持つ
  uint16_t usval[PAD_LEN];    // 上記のunsigned short型
  int8_t bval[PAD_LEN * 2];   // 上記のbyte型
  uint8_t ubval[PAD_LEN * 2]; // 上記のunsigned byte型
  uint64_t ui64val;           // 上記のunsigned int16型
                              // [0]button, [1]pad.stick_L_x:pad.stick_L_y,
                              // [2]pad.stick_R_x:pad.stick_R_y, [3]pad.L2_val:pad.R2_val
} PadUnion;
PadUnion pad_array = {0}; // pad値の格納用配列
PadUnion pad_i2c = {0};   // pad値のi2c送受信用配列

// リモコンのアナログ入力データ
struct PadValue {
  unsigned short stick_R = 0;
  int stick_R_x = 0;
  int stick_R_y = 0;
  unsigned short stick_L = 0;
  int stick_L_x = 0;
  int stick_L_y = 0;
  unsigned short stick_L2R2V = 0;
  int R2_val = 0;
  int L2_val = 0;
};
PadValue pad_analog;

// 6軸or9軸センサーの値
struct AhrsValue {
  Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire); // BNO055のインスタンス

  MPU6050 mpu6050;        // MPU6050のインスタンス
  uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
  uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
  uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
  uint8_t fifoBuffer[64]; // FIFO storage buffer
  Quaternion q;           // [w, x, y, z]         quaternion container
  VectorFloat gravity;    // [x, y, z]            gravity vector
  float ypr[3];           // [roll, pitch, yaw]   roll/pitch/yaw container and gravity vector
  float yaw_origin = 0;   // ヨー軸の補正センター値
  float yaw_source = 0;   // ヨー軸のソースデータ保持用

  float read[16]; // mpuからの読み込んだ一次データacc_x,y,z,gyro_x,y,z,mag_x,y,z,gr_x,y,z,rpy_r,p,y,temp

  float zeros[16] = {0};               // リセット用
  float ave_data[16];                  // 上記の移動平均値を入れる
  float result[16];                    // 加工後の最新のmpuデータ(二次データ)
  float stock_data[IMUAHRS_STOCK][16]; // 上記の移動平均値計算用のデータストック
  int stock_count = 0;                 // 上記の移動平均値計算用のデータストックを輪番させる時の変数
  VectorInt16 aa;                      // [x, y, z]            加速度センサの測定値
  VectorInt16 gyro;                    // [x, y, z]            角速度センサの測定値
  VectorInt16 mag;                     // [x, y, z]            磁力センサの測定値
  long temperature;                    // センサの温度測定値
};
AhrsValue ahrs;

// サーボ用変数
struct ServoParam {
  // サーボの最大接続 (サーボ送受信のループ処理数)
  int num_max;

  // 各サーボのマウントありなし(config.hで設定)
  int ixl_mount[IXL_MAX]; // L系統
  int ixr_mount[IXR_MAX]; // R系統

  // 各サーボのコード上のインデックスに対し, 実際に呼び出すハードウェアのID番号(config.hで設定)
  int ixl_id[IXL_MAX]; // L系統の実サーボ呼び出しID番号
  int ixr_id[IXR_MAX]; // R系統の実サーボ呼び出しID番号

  // 各サーボの正逆方向補正用配列(config.hで設定)
  int ixl_cw[IXL_MAX]; // L系統
  int ixr_cw[IXR_MAX]; // R系統

  // 各サーボの直立ポーズトリム値(config.hで設定)
  float ixl_trim[IXL_MAX]; // L系統
  float ixr_trim[IXR_MAX]; // R系統

  // 各サーボのベンダーと型番(config.hで設定)
  float ixl_type[IXL_MAX]; // L系統
  float ixr_type[IXR_MAX]; // R系統

  // 各サーボのポジション値(degree)
  float ixl_tgt[IXL_MAX] = {0};      // L系統の目標値
  float ixr_tgt[IXR_MAX] = {0};      // R系統の目標値
  float ixl_tgt_past[IXL_MAX] = {0}; // L系統の前回の値
  float ixr_tgt_past[IXR_MAX] = {0}; // R系統の前回の値

  // サーボのエラーカウンタ配列
  int ixl_err[IXL_MAX] = {0}; // L系統
  int ixr_err[IXR_MAX] = {0}; // R系統

  // サーボのコンディションステータス配列
  uint16_t ixl_stat[IXL_MAX] = {0}; // L系統サーボのコンディションステータス配列
  uint16_t ixr_stat[IXR_MAX] = {0}; // R系統サーボのコンディションステータス配列
};
ServoParam sv;

// モニタリング設定
struct MrdMonitor {
  bool flow = MONITOR_FLOW;           // フローを表示
  bool all_err = MONITOR_ERR_ALL;     // 全経路の受信エラー率を表示
  bool servo_err = MONITOR_ERR_SERVO; // サーボエラーを表示
  bool seq_num = MONITOR_SEQ;         // シーケンス番号チェックを表示
  bool pad = MONITOR_PAD;             // リモコンのデータを表示
};
MrdMonitor monitor;

#include "mrd_disp.h"
MrdMsgHandler mrd_disp(Serial);

//==================================================================================================
//  関数各種
//==================================================================================================

///@brief Generate expected sequence number from input.
///@param a_previous_num Previous sequence number.
///@return Expected sequence number. (0 to 59,999)
uint16_t mrd_seq_predict_num(uint16_t a_previous_num) {
  uint16_t x_tmp = a_previous_num + 1;
  if (x_tmp > 59999) // Reset counter
  {
    x_tmp = 0;
  }
  return x_tmp;
}

#endif //__MERIDIAN_MAIN_FUNC__
