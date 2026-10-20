#ifndef __MERIDIAN_MAIN_FUNC__
#define __MERIDIAN_MAIN_FUNC__

// ヘッダファイルの読み込み
#include "config.h"

// ライブラリ導入
#include <Adafruit_BNO055.h>            // 9軸センサBNO055用
#include <MPU6050_6Axis_MotionApps20.h> // MPU6050用
#include <Meridian.h>                   // Meridianのライブラリ導入
extern MERIDIANFLOW::Meridian mrd;
class MrdHalfDuplexUart;         // 半二重UARTバス (mrd_servo.h で定義)
class IcsHalfDuplexClass;        // ICSサーボ (mrd_module/sv_ics.h で定義)
class FeetechHalfDuplexClass;    // FEETECHサーボ (mrd_module/sv_ftc.h で定義)
class Dynamixel2HalfDuplexClass; // DYNAMIXEL 2.0サーボ (mrd_module/sv_dxl2.h で定義)
class FutabaHalfDuplexClass;     // FUTABA RS30xサーボ (mrd_module/sv_ftbrx.h で定義)
class XbusHalfDuplexClass;       // JR XBUSサーボ (mrd_module/sv_xbus.h で定義)
class HitecHalfDuplexClass;      // Hitec HSRサーボ (mrd_module/sv_hitec.h で定義)
class PmxHalfDuplexClass;        // KONDO PMXサーボ (mrd_module/sv_pmx.h で定義)
class Dynamixel1HalfDuplexClass; // DYNAMIXEL 1.0サーボ (mrd_module/sv_dxl1.h で定義)
extern MrdHalfDuplexUart sv_bus_l;
extern MrdHalfDuplexUart sv_bus_r;
extern IcsHalfDuplexClass ics_l;
extern IcsHalfDuplexClass ics_r;
extern FeetechHalfDuplexClass ftc_l;
extern FeetechHalfDuplexClass ftc_r;
extern Dynamixel2HalfDuplexClass dxl2_l;
extern Dynamixel2HalfDuplexClass dxl2_r;
extern FutabaHalfDuplexClass ftb_l;
extern FutabaHalfDuplexClass ftb_r;
extern XbusHalfDuplexClass xbus_l;
extern XbusHalfDuplexClass xbus_r;
extern HitecHalfDuplexClass htc_l;
extern HitecHalfDuplexClass htc_r;
extern PmxHalfDuplexClass pmx_l;
extern PmxHalfDuplexClass pmx_r;
extern Dynamixel1HalfDuplexClass dxl1_l;
extern Dynamixel1HalfDuplexClass dxl1_r;

/// @brief LEDを点灯/消灯する. ピン番号が負(ピンをPWM等に使うためLEDなし)なら何もしない.
inline void mrd_led_write(int a_pin, int a_val) {
  if (a_pin >= 0) {
    digitalWrite(a_pin, a_val);
  }
}

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
  PWM_S = 1,     // 本体ピンのPWM(スロット単位で使用. sv_pwm.h)
  PCA9685 = 11,  // I2C_PCA9685 to PWM(スロット単位で使用. sv_pwm.h)
  FTBRSX = 21,   // FUTABA_RSxTTL (RS30x)
  DXL1 = 31,     // DYNAMIXEL 1.0 (AX/RX/EX/MX)
  DXL2 = 32,     // DYNAMIXEL 2.0 (Xシリーズ)
  KOICS3 = 43,   // KONDO_ICS 3.5 / 3.6
  KOPMX = 44,    // KONDO_PMX (RS-485)
  JRXBUS = 51,   // JRPROPO_XBUS
  FTCSTS = 61,   // FEETECH_STS
  FTCSCS = 62,   // FEETECH_SCS
  HTCHSR = 71    // Hitec_HSR (HMIシリアル)
};

/// @brief UARTを使わないスロットのサーボタイプか(1:本体PWM, 11:PCA9685).
///        このスロットはUARTの系統では扱わず, PWM出力の対応表(sv_pwm.h)から指令値として参照される.
inline bool mrd_servo_is_pwm_type(int a_type) { return a_type == PWM_S || a_type == PCA9685; }

enum ImuAhrsType { // 6軸9軸センサ種の列挙型(NO_IMU, MPU6050_IMU, MPU9250_IMU, BNO055_AHRS)
  NO_IMU = 0,      // IMU/AHRS なし.
  MPU6050_IMU = 1, // MPU6050
  MPU9250_IMU = 2, // MPU9250(未設定)
  BNO055_AHRS = 3  // BNO055
};

enum PadType {   // リモコン種の列挙型(NONE, PC, MERIMOTE, BLUERETRO, SBDBT, KRR5FH, WIIMOTE, WIIMOTE_C)
  NONE = 0,      // リモコンなし
  PC = 0,        // PCからのPD入力情報を使用
  MERIMOTE = 1,  // MERIMOTE(未導入)
  BLUERETRO = 2, // BlueRetro(Wii extension 出力を I2C0 / I2C1 で読む)
  SBDBT = 3,     // SBDBT(RCB3形式のシリアル. PIN_PAD_RX でソフトウェア受信)
  KRR5FH = 4,    // KRR5FH
  WIIMOTE = 5,   // WIIMOTE / WIIMOTE + Nunchuk
  WIIMOTE_C = 6, // WIIMOTE + クラシックコントローラ(/Pro)
};

enum PadButton {  // リモコンボタンの列挙型
  PAD_SELECT = 1, // Select
  PAD_HOME = 2,   // HOME(L3と同じビットを共用)
  PAD_L3 = 2,     // L3
  PAD_R3 = 4,     // R3
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

enum BinHexDec { // 数値表示タイプの列挙型(Bin, Hex, Dec). EEPROM_STYLE で使う
  Bin = 0,       // BIN
  Hex = 1,       // HEX
  Dec = 2,       // DEC
};

//------------------------------------------------------------------------------------
//  設定値
//------------------------------------------------------------------------------------

// 起動時に決まる設定値. 既定値は config.h と keys.h の値. EEPROM_LOAD で EEPROM を使うページを選ぶと,
// 起動の最初にそのページの値で上書きする(mrd_eeprom_settings_boot). コードはマクロではなくこの値を使う.
// 項目の並びは EEPROM のページの順(P0 ネットワーク, P3 サーボの系統, P4 システム, P5 PWM).
// スロットの設定(IXL_MT, IXL_PWM_CENTER など)と PWM の対応表(PWM_PIN_SLOT, PCA9685_SLOT)は config.h の配列をそのまま使う.
struct MrdSettings {
  // P0 ネットワーク(keys.h). 文字列の長さは EEPROM の大きさに合わせる(IPは "a.b.c.d")
  int mode_ether = MODE_ETHER;                  // 0:WiFi, 1:有線LAN
  int mode_fixed_ip = MODE_FIXED_IP;            // WiFiでIPアドレスを固定するか
  int udp_send_port = UDP_SEND_PORT;            // 送り先(PC)のポート番号
  int udp_recv_port = UDP_RECV_PORT;            // このボードのポート番号
  char wifi_send_ip[16] = WIFI_SEND_IP;         // 送り先(PC)のIPアドレス(WiFi)
  char fixed_ip_addr[16] = FIXED_IP_ADDR;       // 固定IPのときのボードのIPアドレス
  char fixed_ip_gateway[16] = FIXED_IP_GATEWAY; // 固定IPのときのゲートウェイ
  char fixed_ip_subnet[16] = FIXED_IP_SUBNET;   // 固定IPのときのサブネットマスク
  char ether_local_ip[16] = ETHER_LOCAL_IP;     // 有線LANのボードのIPアドレス
  char ether_gateway[16] = ETHER_GATEWAY;       // 有線LANの送り先(PC)のIPアドレス
  char ether_subnet[16] = ETHER_SUBNET;         // 有線LANのサブネットマスク
  char ether_dns[16] = ETHER_DNS;               // 有線LANのDNS
  char ether_mac[18] = ETHER_MAC;               // 有線LANのMACアドレス("XX:XX:XX:XX:XX:XX")
  char wifi_ap_ssid[33] = WIFI_AP_SSID;         // WiFiのアクセスポイントのSSID(32byteまで)
  char wifi_ap_pass[65] = WIFI_AP_PASS;         // WiFiのアクセスポイントのパスワード(64byteまで)

  // P3 サーボの系統と共通設定
  int mount_servo_type_l = MOUNT_SERVO_TYPE_L;                 // L系統のサーボの種類
  int mount_servo_type_r = MOUNT_SERVO_TYPE_R;                 // R系統のサーボの種類
  int mount_servo_type_l2 = MOUNT_SERVO_TYPE_L2;               // L系統の2種類目(0:混在なし)
  int mount_servo_type_r2 = MOUNT_SERVO_TYPE_R2;               // R系統の2種類目(0:混在なし)
  unsigned long servo_baudrate_l = SERVO_BAUDRATE_L;           // L系統の通信速度(bps)
  unsigned long servo_baudrate_r = SERVO_BAUDRATE_R;           // R系統の通信速度(bps)
  unsigned long servo_baudrate_l2 = SERVO_BAUDRATE_L2;         // L系統の2種類目の通信速度(bps)
  unsigned long servo_baudrate_r2 = SERVO_BAUDRATE_R2;         // R系統の2種類目の通信速度(bps)
  int servo_timeout_l = SERVO_TIMEOUT_L;                       // L系統の返信待ちのタイムアウト(ms)
  int servo_timeout_r = SERVO_TIMEOUT_R;                       // R系統の返信待ちのタイムアウト(ms)
  int servo_echo_l = SERVO_ECHO_L;                             // L系統の送信エコー処理(0:なし, 1:あり, 2:自動)
  int servo_echo_r = SERVO_ECHO_R;                             // R系統の送信エコー処理
  int servo_tx_release = SERVO_TX_RELEASE;                     // 受信中にTXピンの出力を切るか
  int servo_lost_err_wait = SERVO_LOST_ERR_WAIT;               // 通信不能とみなす連続失敗の回数
  float servo_step_max = SERVO_STEP_MAX;                       // 1フレームの指令値の最大変化量(degree)
  int servo_unmount_reply = SERVO_UNMOUNT_REPLY;               // マウントしていないスロットの返信(0:0, 1:受信値)
  int servo_motion_frames_max = SERVO_MOTION_FRAMES_MAX;       // Meridim[19](補間フレーム数)の上限
  float servo_ftc_sts_range = SERVO_FTC_STS_RANGE;             // FEETECH STS の角度範囲(degree)
  float servo_ftc_scs_range = SERVO_FTC_SCS_RANGE;             // FEETECH SCS の角度範囲(degree)
  int servo_ftc_scs_speed = SERVO_FTC_SCS_SPEED;               // FEETECH SCS の目標速度(位置値/s)
  int servo_ftc_fix_return_delay = SERVO_FTC_FIX_RETURN_DELAY; // FEETECH の応答遅延時間を起動時に0にするか
  int servo_dxl_fix_return_delay = SERVO_DXL_FIX_RETURN_DELAY; // DYNAMIXEL の応答遅延時間を起動時に0にするか
  int servo_xbus_angle180 = SERVO_XBUS_ANGLE180;               // XBUS の180度モード
  int servo_htc_per_frame = SERVO_HTC_PER_FRAME;               // Hitec の1フレームに送る/読む台数
  float servo_htc_us_per_deg = SERVO_HTC_US_PER_DEG;           // Hitec の1degあたりのパルス幅(us)
  int servo_htc_speed = SERVO_HTC_SPEED;                       // Hitec の速度(1-255)
  int servo_ftb_read_per_frame = SERVO_FTB_READ_PER_FRAME;     // FUTABA の1フレームに位置を読む台数

  // P4 システム・動作モード
  int frame_duration = FRAME_DURATION;                       // 1フレームの時間(ms)
  int charge_time = CHARGE_TIME;                             // 起動時のコンデンサの充電待ち(ms)
  int mount_sd = MOUNT_SD;                                   // SDカードの有無
  int mount_imuahrs = MOUNT_IMUAHRS;                         // IMU/AHRS の種類(ImuAhrsType)
  int mount_pad = MOUNT_PAD;                                 // ジョイパッドの種類(PadType)
  int mode_esp32_standalone = MODE_ESP32_STANDALONE;         // ボード単体モード
  int mode_udp_receive = MODE_UDP_RECEIVE;                   // PCからの受信
  int mode_udp_send = MODE_UDP_SEND;                         // PCへの送信
  int mode_udp_passive = MODE_UDP_PASSIVE;                   // 通信の周期(0:ボード主導, 1:PC主導)
  int udp_timeout_us = UDP_TIMEOUT_US;                       // 受信待ちのタイムアウト(us)
  int udp_poll_us = UDP_POLL_US;                             // 受信待ち中の確認間隔(us)
  int udp_failsafe_ms = UDP_FAILSAFE_MS;                     // 受信が途絶えてから脱力するまで(ms. 0:脱力しない)
  float udp_self_speed = UDP_SELF_SPEED;                     // 受信しないモードの速さの上限(degree/s)
  int check_sd_rw = CHECK_SD_RW;                             // 起動時のSDカードの読み書きチェック
  int monitor_frame_delay = MONITOR_FRAME_DELAY;             // 処理時間と遅延の表示
  int monitor_flow = MONITOR_FLOW;                           // 処理の段階の表示
  int monitor_err_all = MONITOR_ERR_ALL;                     // 全経路の受信エラーの表示
  int monitor_seq = MONITOR_SEQ;                             // シーケンス番号の表示
  int monitor_udp_timeout = MONITOR_UDP_TIMEOUT;             // 受信のタイムアウトの表示
  int monitor_suppress_duration = MONITOR_SUPPRESS_DURATION; // 起動直後にタイムアウトの表示を抑える時間(ms)
  unsigned long i2c0_speed = I2C0_SPEED;                     // I2C(IMU/AHRS)の速度(Hz)
  int imuahrs_interval = IMUAHRS_INTERVAL;                   // IMU/AHRS の読み取り間隔(ms)
  int mount_ina226 = MOUNT_INA226;                           // INA226 の搭載(0:なし, 1:あり)
  int ina226_addr = INA226_ADDR;                             // INA226 のI2Cアドレス
  float ina226_shunt_mohm = INA226_SHUNT_MOHM;               // INA226 のシャント抵抗(mΩ)
  int ina226_interval = INA226_INTERVAL;                     // INA226 の読み取り間隔(ms)
  unsigned long serial_pc_bps = SERIAL_PC_BPS;               // USBシリアルの速度(bps)
  int pad_init_timeout = PAD_INIT_TIMEOUT;                   // 起動時のジョイパッドの接続待ち(ms)
  int pad_interval = PAD_INTERVAL;                           // ジョイパッドを読む間隔(ms)
  int pad_button_marge = PAD_BUTTON_MARGE;                   // ボタンを受信値に論理和で重ねるか
  int pad_generalize = PAD_GENERALIZE;                       // ボタンをPS系の配置に一般化するか
  unsigned long pad_sbdbt_bps = PAD_SBDBT_BPS;               // SBDBT の通信速度(bps)
  int pad_sbdbt_timeout = PAD_SBDBT_TIMEOUT;                 // SBDBT の電文が途絶えたらパッドの値を0にする時間(ms. 0:しない)
  int pad_blueretro_i2c = PAD_BLUERETRO_I2C;                 // BlueRetro をつなぐI2C(0:I2C0, 1:I2C1)

  // P5 PWM の共通設定(スロットごとの設定 P6/P7 と対応表は config.h の配列)
  int pwm_pin_mode = PWM_PIN_MODE;               // ピン25/26の用途(0:LED, 1:本体PWM, 2:PCA9685)
  unsigned long i2c1_speed = I2C1_SPEED;         // I2C1(Wire1. PCA9685, BlueRetro)の速度(Hz)
  int pca9685_num = PCA9685_NUM;                 // PCA9685の枚数(0-2)
  int pca9685_addr_0 = PCA9685_ADDR_0;           // 1枚目のI2Cアドレス
  int pca9685_addr_1 = PCA9685_ADDR_1;           // 2枚目のI2Cアドレス
  unsigned long pca9685_osc_hz = PCA9685_OSC_HZ; // PCA9685の発振器の周波数(Hz)

  // P1 で EEPROM がマウントを外したスロット(1:外す). config のマウント(種類が0でもPWM専用でもない)と合わせて決まる
  int ixl_unmount[IXL_MAX] = {0}; // L系統
  int ixr_unmount[IXR_MAX] = {0}; // R系統
};
MrdSettings cfg;

/// @brief ピンを LED 以外に使うか: 本体PWM(PWM_PIN_MODE 1), I2C1(PCA9685 の PWM_PIN_MODE 2, BlueRetro),
///        SBDBT の受信(PIN_PAD_RX). BlueRetro は I2C1 につないだとき(PAD_BLUERETRO_I2C 1)だけ.
inline bool mrd_pin_used_for_other(int a_pin) {
  if (cfg.pwm_pin_mode == 1 && (a_pin == PIN_PWM_0 || a_pin == PIN_PWM_1)) {
    return true;
  }
  const bool blueretro_i2c1_tmp = (cfg.mount_pad == BLUERETRO && cfg.pad_blueretro_i2c == 1);
  if ((cfg.pwm_pin_mode == 2 || blueretro_i2c1_tmp) && (a_pin == PIN_I2C1_SDA || a_pin == PIN_I2C1_SCL)) {
    return true;
  }
  return cfg.mount_pad == SBDBT && a_pin == PIN_PAD_RX;
}

/// @brief エラーLEDのピン(PIN_ERR_LED). そのピンを LED 以外に使うときは LED なし(-1).
inline int mrd_pin_err_led() { return (PIN_ERR_LED >= 0 && !mrd_pin_used_for_other(PIN_ERR_LED)) ? PIN_ERR_LED : -1; }

/// @brief Bluetooth接続確認用LEDのピン(PIN_LED_BT). そのピンを LED 以外に使うときは LED なし(-1).
inline int mrd_pin_led_bt() { return (PIN_LED_BT >= 0 && !mrd_pin_used_for_other(PIN_LED_BT)) ? PIN_LED_BT : -1; }

//------------------------------------------------------------------------------------
//  変数
//------------------------------------------------------------------------------------

// システム用の変数
const int MRDM_BYTE = MRDM_LEN * 2; // Meridim配列のバイト型の長さ
const int MRD_ERR = MRDM_LEN - 2;   // エラーフラグの格納場所(配列の末尾から2つめ)
const int MRD_ERR_l = MRD_ERR * 2;  // エラーフラグの格納場所(下位8ビット)
const int MRD_CKSM = MRDM_LEN - 1;  // チェックサムの格納場所(配列の末尾)
const int PAD_LEN = 5;              // リモコン用配列の長さ
TaskHandle_t thp[4];                // マルチスレッドのタスクハンドル格納用

//------------------------------------------------------------------------------------
//  クラス・構造体・共用体
//------------------------------------------------------------------------------------

// Meridim配列用の共用体の設定
typedef union {
  short sval[MRDM_LEN + 4];           // short型でMRDM_LEN個の配列データを持つ(+4は余白)
  unsigned short usval[MRDM_LEN + 2]; // 上記のunsigned short型(+2は余白)
  uint8_t bval[MRDM_BYTE + 4];        // byte型でMRDM_BYTE個の配列データを持つ
  uint8_t ubval[MRDM_BYTE + 4];       // 上記のunsigned byte型
} Meridim90Union;
Meridim90Union s_udp_meridim;      // Meridim配列データ送信用(short型, センサや角度は100倍値)
Meridim90Union r_udp_meridim;      // Meridim配列データ受信用
Meridim90Union r_udp_meridim_last; // PCから最後に正しく受信したMeridim(受信できない・チェックサム不正のフレームはこれを使う)

// フラグ用変数
struct MrdFlags {
  bool udp_board_passive = cfg.mode_udp_passive; // 通信の周期がPC主導か(false:ボード主導). 10005/10006 で切り替え
  bool count_frame_reset = false;                // フレーム管理時計をリセットする
  bool eeprom_write_mode = false;                // EEPROMへの書き込みモード. ※未実装(参照する処理なし)

  bool udp_receive_mode = cfg.mode_udp_receive; // PCからのデータ受信実施(0:OFF, 1:ON, 通常は1)
  bool udp_send_mode = cfg.mode_udp_send;       // PCへのデータ送信実施(0:OFF, 1:ON, 通常は1)
  bool meridim_rcvd = false;                    // Meridimが正しく受信できたか.
  bool udp_failsafe = false;                    // PCからの受信が途絶えて全サーボを脱力しているか(フェイルセーフ).
};
MrdFlags flg;

// シーケンス番号管理用の変数
struct MrdSq {
  int s_increment = 0; // フレーム毎に0-59999をカウントし, 送信
  int r_expect = 0;    // フレーム毎に0-59999をカウントし, 受信値と比較
};
MrdSq mrdsq;

// タイマー管理用の変数
struct MrdTimer {
  int count_loop = 0;          // サイン計算用の循環カウンタ
  int count_loop_dlt = 2;      // サイン計算用の循環カウンタを1フレームにいくつ進めるか
  int count_loop_max = 359999; // 循環カウンタの最大値
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
  short sval[PAD_LEN];        // short型でPAD_LEN個の配列データを持つ
  uint16_t usval[PAD_LEN];    // 上記のunsigned short型
  int8_t bval[PAD_LEN * 2];   // 上記のbyte型
  uint8_t ubval[PAD_LEN * 2]; // 上記のunsigned byte型
  uint64_t ui64val;           // 先頭64bit(usval[0]-[3])をまとめて扱う
                              // [0]button, [1]pad.stick_L_x:pad.stick_L_y,
                              // [2]pad.stick_R_x:pad.stick_R_y, [3]pad.L2_val:pad.R2_val
} PadUnion;
PadUnion pad_array = {0}; // pad値の格納用配列

// 6軸or9軸センサーの値
struct AhrsValue {
  Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire); // BNO055のインスタンス

  MPU6050 mpu6050;         // MPU6050のインスタンス
  uint8_t dev_status;      // return status after each device operation (0 = success, !0 = error)
  uint16_t packet_size;    // expected DMP packet size (default is 42 bytes)
  uint8_t fifo_buffer[64]; // FIFO storage buffer
  Quaternion q;            // [w, x, y, z]         quaternion container
  VectorFloat gravity;     // [x, y, z]            gravity vector
  float ypr[3];            // [yaw, pitch, roll]   yaw/pitch/roll container
  float yaw_origin = 0;    // ヨー軸の補正センター値
  float yaw_source = 0;    // ヨー軸のソースデータ保持用
  float roll_origin = 0;   // ロール軸の補正センター値(MCMD_SENSOR_ALL_CALIB)
  float roll_source = 0;   // ロール軸のソースデータ保持用
  float pitch_origin = 0;  // ピッチ軸の補正センター値(MCMD_SENSOR_ALL_CALIB)
  float pitch_source = 0;  // ピッチ軸のソースデータ保持用

  float read[16]; // センサから読み込んだ一次データacc_x,y,z,gyro_x,y,z,mag_x,y,z,gr_x,y,z,rpy_r,p,y,temp

  VectorInt16 aa;   // [x, y, z]            加速度センサの測定値
  VectorInt16 gyro; // [x, y, z]            角速度センサの測定値
};
AhrsValue ahrs;

// サーボ用変数. 各配列は IXL_MAX / IXR_MAX の長さで, Meridim の1系統のスロット数(MRD_SERVO_SLOTS)と同じであること
static_assert(IXL_MAX == MRD_SERVO_SLOTS && IXR_MAX == MRD_SERVO_SLOTS, "IXL_MAX and IXR_MAX must equal MRD_SERVO_SLOTS");
struct ServoParam {
  // サーボの最大接続 (サーボ送受信のループ処理数)
  int num_max;

  // 各サーボのマウントありなし(config.h または EEPROM の P1/P2 で設定)
  int ixl_mount[IXL_MAX]; // L系統
  int ixr_mount[IXR_MAX]; // R系統

  // 各サーボのコード上のインデックスに対し, 実際に呼び出すハードウェアのID番号(config.h または EEPROM の P1/P2 で設定)
  int ixl_id[IXL_MAX]; // L系統の実サーボ呼び出しID番号
  int ixr_id[IXR_MAX]; // R系統の実サーボ呼び出しID番号

  // 各サーボの正逆方向補正用配列(config.h または EEPROM の P1 で設定)
  int ixl_cw[IXL_MAX]; // L系統
  int ixr_cw[IXR_MAX]; // R系統

  // 各サーボの直立ポーズトリム値(config.h または EEPROM の P1 で設定)
  float ixl_trim[IXL_MAX]; // L系統
  float ixr_trim[IXR_MAX]; // R系統

  // 各サーボの種類(IXL_MT/IXR_MT の番号. config.h または EEPROM の P2 で設定)
  float ixl_type[IXL_MAX]; // L系統
  float ixr_type[IXR_MAX]; // R系統

  // 各サーボのポジション値(degree)
  float ixl_tgt[IXL_MAX] = {0};      // L系統の目標値
  float ixr_tgt[IXR_MAX] = {0};      // R系統の目標値
  float ixl_tgt_past[IXL_MAX] = {0}; // L系統の前回の値
  float ixr_tgt_past[IXR_MAX] = {0}; // R系統の前回の値
  float ixl_cmd_past[IXL_MAX] = {0}; // L系統の前回の指令値(移動量制限の基準)
  float ixr_cmd_past[IXR_MAX] = {0}; // R系統の前回の指令値(移動量制限の基準)

  // サーボのエラーカウンタ配列
  int ixl_err[IXL_MAX] = {0}; // L系統
  int ixr_err[IXR_MAX] = {0}; // R系統

  // サーボのコンディションステータス配列
  uint16_t ixl_stat[IXL_MAX] = {0}; // L系統サーボのコンディションステータス配列
  uint16_t ixr_stat[IXR_MAX] = {0}; // R系統サーボのコンディションステータス配列
};
ServoParam sv;

// フレーム処理時間の集計用(MONITOR_FRAME_DELAY)
struct MrdFrameMon {
  unsigned long busy_sum = 0;                    // フレーム処理時間(ループ開始から終端の待機まで)の合計(us)
  unsigned long busy_max = 0;                    // フレーム処理時間の最大(us)
  unsigned long servo_sum = 0;                   // サーボ通信時間の合計(us)
  unsigned long servo_max = 0;                   // サーボ通信時間の最大(us)
  int late = 0;                                  // 処理が1フレームの時間を超えたフレーム数
  int count = 0;                                 // 集計したフレーム数
  uint8_t slot_fail[2][MRD_SERVO_SLOTS] = {{0}}; // サーボごとの通信失敗フレーム数([0]:L, [1]:R)
};
MrdFrameMon frame_mon;

// 応答待ち時間の集計(MONITOR_FRAME_DELAYの表示用. mrd_servo_lines_run で集計)
// 系統ごとに, 受信切替から受信完了までの時間(系統ごとの応答の速さ).
struct MrdHduartStat {
  uint32_t sum_us[2] = {0, 0};
  uint32_t max_us[2] = {0, 0};
  uint32_t count[2] = {0, 0};
  uint16_t fail[2][5] = {{0}}; // 系統ごとの受信失敗の種類別回数(MrdHalfDuplexUart::RxResult)
};
MrdHduartStat hduart_stat;

#include "mrd_disp.h"
MrdMsgHandler mrd_disp(Serial);

//==================================================================================================
//  関数各種
//==================================================================================================

///@brief Generate expected sequence number from input.
///@param a_previous_num Previous sequence number.
///@return Expected sequence number. (0 to 59,999)
uint16_t mrd_seq_num_predict(uint16_t a_previous_num) {
  uint16_t x_tmp = a_previous_num + 1;
  if (x_tmp > 59999) // Reset counter
  {
    x_tmp = 0;
  }
  return x_tmp;
}

#endif //__MERIDIAN_MAIN_FUNC__
