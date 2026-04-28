#ifndef __MERIDIAN_COMMON_H__
#define __MERIDIAN_COMMON_H__

// Meridianモジュール間で共有される共通の型と定義
// このファイルはモジュールの自己完結性のため最小限の依存関係を提供する
// 注意: config.h はここでインクルードしない. main.cpp からのみインクルードすること.

#include <stdint.h>

//------------------------------------------------------------------------------------
//  コア定数 (全モジュール共通)
//  これらの値は, このファイルより先に config.h がインクルードされた場合上書き可能
//------------------------------------------------------------------------------------

// Meridim配列の設定
#ifndef MRDM_LEN
#define MRDM_LEN 90 // Meridim配列の長さ (デフォルト90)
#endif
#define MRDM_BYTE (MRDM_LEN * 2)
#define MRD_CKSM (MRDM_LEN - 1)

// サーボシステムの上限
#ifndef IXL_MAX
#define IXL_MAX 15 // L系統の最大サーボ数
#endif
#ifndef IXR_MAX
#define IXR_MAX 15 // R系統の最大サーボ数
#endif

// EEPROMデフォルト値 (main.cpp がインクルードする前に config.h で上書き可能)
#ifndef EEPROM_SIZE
#define EEPROM_SIZE 540 // 使用するEEPROMのサイズ(バイト)
#endif
#ifndef EEPROM_PROTECT
#define EEPROM_PROTECT 0
#endif
#ifndef EEPROM_LOAD
#define EEPROM_LOAD 1
#endif
#ifndef EEPROM_SET
#define EEPROM_SET 0
#endif

// UDPモードデフォルト値
#ifndef MODE_UDP_RECEIVE
#define MODE_UDP_RECEIVE 1
#endif
#ifndef MODE_UDP_SEND
#define MODE_UDP_SEND 1
#endif

//------------------------------------------------------------------------------------
//  列挙型
//------------------------------------------------------------------------------------

// サーボ系統識別子 (L, R, C)
enum UartLine {
  L, // 左
  R, // 右
  C  // 中央
};

// パッド/コントローラの種類
enum PadType {
  NONE = 0,      // コントローラなし
  PC = 0,        // PC入力
  MERIMOTE = 1,  // MERIMOTE
  BLUERETRO = 2, // BLUERETRO
  SBDBT = 3,     // SBDBT
  KRR5FH = 4,    // KRR5FH
  WIIMOTE = 5,   // WIIMOTE / WIIMOTE + ヌンチャク
  WIIMOTE_C = 6, // WIIMOTE+クラシックコントローラ
};

// サーボプロトコルの種類
enum ServoType {
  NOSERVO = 0,  // サーボなし
  PWM_S = 1,    // 単体PWM (WIP)
  PCA9685 = 11, // I2C_PCA9685 to PWM (WIP)
  FTBRSX = 21,  // FUTABA_RSxTTL (WIP)
  DXL1 = 31,    // DYNAMIXEL 1.0 (WIP)
  DXL2 = 32,    // DYNAMIXEL 2.0 (WIP)
  KOICS3 = 43,  // KONDO_ICS 3.5 / 3.6
  KOPMX = 44,   // KONDO_PMX (WIP)
  JRXBUS = 51,  // JRPROPO_XBUS (WIP)
  FTCSTS = 61,  // FEETECH_STS (WIP)
  FTCSCS = 62   // FEETECH_SCS (WIP)
};

// IMU/AHRSセンサの種類
enum ImuAhrsType {
  NO_IMU = 0,      // IMU/AHRSなし
  MPU6050_IMU = 1, // MPU6050
  MPU9250_IMU = 2, // MPU9250 (WIP)
  BNO055_AHRS = 3  // BNO055
};

//------------------------------------------------------------------------------------
//  定数
//------------------------------------------------------------------------------------

const int PAD_LEN = 5; // リモコン配列の長さ

//------------------------------------------------------------------------------------
//  構造体
//------------------------------------------------------------------------------------

// エラーカウント構造体
struct MrdErr {
  int esp_pc = 0;   // PC受信エラー (ESP32からのUDP)
  int pc_esp = 0;   // ESP32受信エラー (PCからのUDP)
  int esp_tsy = 0;  // Teensy受信エラー (ESP32からのSPI)
  int tsy_esp = 0;  // ESP32受信エラー (TeensyからのSPI)
  int esp_skip = 0; // UDP→ESPフレームスキップ数
  int tsy_skip = 0; // ESP→Teensyフレームスキップ数
  int pc_skip = 0;  // PCフレームスキップ数
};

// サーボパラメータ構造体
struct ServoParam {
  // 最大サーボ接続数 (サーボ送受信ループ回数)
  int num_max;

  // サーボマウント状態 (config.h で設定)
  int ixl_mount[IXL_MAX]; // L系統
  int ixr_mount[IXR_MAX]; // R系統

  // 各サーボインデックスのハードウェアID (config.h で設定)
  int ixl_id[IXL_MAX]; // L系統の実サーボID
  int ixr_id[IXR_MAX]; // R系統の実サーボID

  // 回転方向補正 (config.h で設定)
  int ixl_cw[IXL_MAX]; // L系統
  int ixr_cw[IXR_MAX]; // R系統

  // 直立姿勢用トリム値 (config.h で設定)
  float ixl_trim[IXL_MAX]; // L系統
  float ixr_trim[IXR_MAX]; // R系統

  // サーボのベンダーとモデル (config.h で設定)
  int ixl_type[IXL_MAX]; // L系統
  int ixr_type[IXR_MAX]; // R系統

  // サーボ位置値 (度)
  float ixl_tgt[IXL_MAX] = {0};      // L系統目標値
  float ixr_tgt[IXR_MAX] = {0};      // R系統目標値
  float ixl_tgt_past[IXL_MAX] = {0}; // L系統前回値
  float ixr_tgt_past[IXR_MAX] = {0}; // R系統前回値

  // サーボエラーカウンタ配列
  int ixl_err[IXL_MAX] = {0}; // L系統
  int ixr_err[IXR_MAX] = {0}; // R系統

  // サーボ状態ステータス配列
  uint16_t ixl_stat[IXL_MAX] = {0}; // L系統
  uint16_t ixr_stat[IXR_MAX] = {0}; // R系統
};

// システム状態管理用フラグ
struct MrdFlags {
  bool imuahrs_available = true;        // サブスレッド書込み用センサ読取りロック
  bool udp_board_passive = false;       // UDPタイミング: ボード主導(false) または PC主導(true)
  bool count_frame_reset = false;       // フレーム管理タイマーのリセット
  bool stop_board_during = false;       // meridim[2]秒, meridim[3]ミリ秒間ボード処理を停止
  bool eeprom_write_mode = false;       // EEPROM書き込みモード
  bool eeprom_read_mode = false;        // EEPROM読み出しモード
  bool eeprom_protect = EEPROM_PROTECT; // EEPROM書き込み保護
  bool eeprom_load = EEPROM_LOAD;       // 起動時にEEPROM内容をロード
  bool eeprom_set = EEPROM_SET;         // 起動時にデフォルト値をEEPROMにセット
  bool sdcard_write_mode = false;       // SDカード書き込みモード
  bool sdcard_read_mode = false;        // SDカード読み出しモード
  bool wire0_init = false;              // I2Cバス0初期化状態
  bool wire1_init = false;              // I2Cバス1初期化状態
  bool bt_busy = false;                 // Bluetooth受信中フラグ (UDP競合回避)
  bool spi_rcvd = true;                 // SPIデータ受信状態
  bool udp_rcvd = false;                // UDPデータ受信状態
  bool udp_busy = false;                // UDPスレッド受信中フラグ (送信抑制)

  bool udp_receive_mode = MODE_UDP_RECEIVE; // PCデータ受信 (0:OFF, 1:ON, デフォルト1)
  bool udp_send_mode = MODE_UDP_SEND;       // PCデータ送信 (0:OFF, 1:ON, デフォルト1)
  bool meridim_rcvd = false;                // Meridim受信成功状態
};

//------------------------------------------------------------------------------------
//  共用体
//------------------------------------------------------------------------------------

// データ送信用Meridim配列共用体
typedef union {
  short sval[MRDM_LEN + 4];           // MRDM_LEN+4要素のshort配列
  unsigned short usval[MRDM_LEN + 2]; // unsigned short配列
  uint8_t bval[MRDM_BYTE + 4];        // バイト配列
  uint8_t ubval[MRDM_BYTE + 4];       // unsigned バイト配列
} Meridim90Union;

// リモコンデータ共用体
typedef union {
  short sval[PAD_LEN];        // PAD_LEN要素のshort配列
  uint16_t usval[PAD_LEN];    // unsigned short配列
  int8_t bval[PAD_LEN * 2];   // バイト配列
  uint8_t ubval[PAD_LEN * 2]; // unsigned バイト配列
  uint64_t ui64val;           // 64ビット unsigned int
                              // [0]ボタン, [1]stick_L_x:stick_L_y,
                              // [2]stick_R_x:stick_R_y, [3]L2_val:R2_val
} PadUnion;

// EEPROM読み書き用共用体
typedef union {
  uint8_t bval[EEPROM_SIZE]; // 1バイト単位のデータ (デフォルト540バイト)
  int16_t saval[3][90];      // short型の3*90配列データ
  uint16_t usaval[3][90];    // unsigned short型の3*90配列データ
  int16_t sval[270];         // 270個のshort型データ
  uint16_t usval[270];       // 270個のunsigned short型データ
} UnionEEPROM;

#endif // __MERIDIAN_COMMON_H__
