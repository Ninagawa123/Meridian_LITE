#ifndef __MERIDIAN_CONFIG__
#define __MERIDIAN_CONFIG__

// ヘッダファイルの読み込み
#include "mrd_types.h"

// ライブラリ導入

//==================================================================================================
//  MERIDIAN - LITE - ESP32の配線
//==================================================================================================
//
// ESP32devkitC  -  デバイス
//   3V3         -  BNO005 VIN
//   21          -  BNO005 SCL
//   22          -  BNO005 SDA
//   GND         -  BNO005 GND
//
//   4  EN       -  ICS変換基板 R系統 EN
//   16 RX       -  ICS変換基板 R系統 TX
//   17 TX       -  ICS変換基板 R系統 RX
//   5V          -  ICS変換基板 IOREF
//   GND         -  ICS変換基板 GND
//
//   33 EN       -  ICS変換基板 L系統 EN
//   32 RX       -  ICS変換基板 L系統 TX
//   27 TX       -  ICS変換基板 L系統 RX
//   5V          -  ICS変換基板 IOREF
//   GND         -  ICS変換基板 GND
//
//   19          -  SPI_MISO
//   23          -  SPI_MOSI
//   18          -  SPI_CSK
//   15          -  SPI_CS SD(起動時LOW必須, 外部PU厳禁)
//   05          -  SPI_CS BOARD PIN(Etherなど)
//
//   00          -  NG/起動時HIGH必須
//   01          -  NG(USBシリアルTX0)
//   02          -  NG/起動時HIGH必須
//   03          -  NG(USBシリアルRX0)
//   06          -  NG(FLASH SCK)
//   07          -  NG(FLASH D0)
//   08          -  NG(FLASH D1)
//   09          -  NG(FLASH HD)
//   10          -  NG(FLASH WP)
//   11          -  NG(FLASH CS)
//   12          -  NG(起動時LOW必須/電圧設定1.8Vor3.3V)
//   13          -  OK/LED?
//   14          -  OK/(SPI HS CLK)
//   25          -  OK/BOARD PIN(Input/Output)/DAC出力0-3.3V
//   26          -  OK/BOARD PIN(Input/Output)/DAC出力0-3.3V
//   34          -  OK/BOARD PIN, アナログ入力のみ（PD/PD/出力不可）
//   35          -  OK/BOARD PIN, アナログ入力のみ（PD/PD/出力不可）
//   39          -  OK/アナログ入力のみ（PD/PD/出力不可）
//   36          -  OK/アナログ入力のみ（PD/PD/出力不可）

//==================================================================================================
//  サーボIDとロボット部位, 軸との対応表 (KHR-3HVの例)
//==================================================================================================
//
// ID    Parts/Axis <ICS_Left_Upper SIO1,SIO2>
// [L00] 頭/ヨー
// [L01] 左肩/ピッチ
// [L02] 左肩/ロール
// [L03] 左肘/ヨー
// [L04] 左肘/ピッチ
// [L05] -
// ID    Parts/Axis <ICS_Left_Lower SIO3,SIO4>
// [L06] 左股/ロール
// [L07] 左股/ピッチ
// [L08] 左膝/ピッチ
// [L09] 左足首/ピッチ
// [L10] 左足首/ロール
// ID    Parts/Axis  <ICS_Right_Upper SIO5,SIO6>
// [R00] 腰/ヨー
// [R01] 右肩/ピッチ
// [R02] 右肩/ロール
// [R03] 右肘/ヨー
// [R04] 右肘/ピッチ
// [R05] -
// ID    Parts/Axis  <ICS_Right_Lower SIO7,SIO8>
// [R06] 右股/ロール
// [R07] 右股/ピッチ
// [R08] 右膝/ピッチ
// [R09] 右足首/ピッチ
// [R10] 右足首/ロール

//==================================================================================================
//  Meridim90配列 一覧表
//==================================================================================================
//
// [00]      マスターコマンド デフォルトは90 で配列数も同時に示す
// [01]      シーケンス番号
// [02]-[04] IMU/AHRS:acc_x,acc_y,acc_z      加速度x,y,z
// [05]-[07] IMU/AHRS:gyro_x,gyro_y,gyro_z   ジャイロx,y,z
// [08]-[10] IMU/AHRS:mag_x,mag_y,mag_z      磁気コンパスx,y,z
// [11]      IMU/AHRS:temp                   温度
// [12]-[14] IMU/AHRS:DMP ROLL,PITCH,YAW     DMP推定値 ロール,ピッチ,ヨー
// [15]      ボタンデータ1
// [16]      ボタンアナログ1(Stick Left)
// [17]      ボタンアナログ2(Stick Right)
// [18]      ボタンアナログ3(L2,R2 アナログ)
// [19]      モーション設定(フレーム数)
// [20]      サーボID LO  コマンド
// [21]      サーボID LO  データ値
// [...]     ...
// [48]      サーボID L14 コマンド
// [49]      サーボID L14 データ値
// [50]      サーボID RO  コマンド
// [51]      サーボID RO  データ値
// [...]     ...
// [78]      サーボID R14 コマンド
// [79]      サーボID R14 データ値
// [80]-[MRDM_LEN-3] free (Meridim90では[87]まで)
// [MRDM_LEN-2] ERROR CODE
// [MRDM_LEN-1] チェックサム

//-------------------------------------------------------------------------
//  各種設定
//-------------------------------------------------------------------------

// Meridimの基本設定
#define MRDM_LEN        90  // Meridim配列の長さ設定(デフォルトは90)
#define FRAME_DURATION  10  // 1フレームあたりの単位時間(単位ms, デフォルトは10)
#define CHARGE_TIME     200 // 起動時のコンデンサチャージ待機時間(単位ms)
#define MRD_L_ORIGIDX   20  // Meridim配列のL系統の最初のインデックス(デフォルトは20)
#define MRD_R_ORIGIDX   50  // Meridim配列のR系統の最初のインデックス(デフォルトは50)
#define MRD_SERVO_SLOTS 15  // Meridim配列の1系統あたりの最大接続サーボ数(デフォルトは15)

// 各種ハードウェアのマウント有無
#define MOUNT_SD      1              // SDカードリーダーの有無(0:なし, 1:あり)
#define MOUNT_IMUAHRS IMUAHRS_BNO055 // IMU/AHRSの搭載 (0:なし, 1:MPU6050, 2:MPU9250, 3:BNO055)

// 動作モード
#define MODE_ESP32_STANDALONE 0 // ESP32をボードに挿さず動作確認(0:NO, 1:YES)
#define MODE_UDP_RECEIVE      1 // PCからのデータ受信(0:OFF, 1:ON, 通常は1)
#define MODE_UDP_SEND         1 // PCへのデータ送信(0:OFF, 1:ON, 通常は1)

// Wifi/有線LANの設定(SSID, パスワード, 固定IP, MACアドレス等は別途keys.hで指定)
#define MODE_ETHER    0 // WiFiか有線LANか(0:wifi, 1:有線LAN, 通常は0)
#define MODE_FIXED_IP 0 // WiFi用IPアドレスを固定するか(0:NO, 1:YES)
#define UDP_TIMEOUT   4 // UDPの待受タイムアウト(単位ms,推奨値0)

// EEPROMの設定
#define EEPROM_SIZE    540            // 使用するEEPROMのサイズ(バイト)
#define EEPROM_SET     0              // 起動時にEEPROMにconfig.hの内容をセット(mrd_set_eeprom)
#define EEPROM_PROTECT 0              // EEPROMの書き込み保護(0:保護しない, 1:書き込み禁止)
#define EEPROM_LOAD    1              // 起動時にEEPROMの内容を諸設定にロードする
#define EEPROM_DUMP    0              // 起動時のEEPROMデータのダンプ表示
#define EEPROM_STYLE   VALUE_TYPE_DEC // 起動時のEEPROM内容のダンプ表示の書式(VALUE_TYPE_BIN,VALUE_TYPE_HEX,VALUE_TYPE_DEC)

// 動作チェックモード
#define CHECK_SD_RW     1 // 起動時のSDカードリーダーの読み書きチェック
#define CHECK_EEPROM_RW 0 // 起動時のEEPROMの動作チェック

// シリアルモニタリング
#define MONITOR_FRAME_DELAY       1    // シリアルモニタでフレーム遅延時間を表示(0:OFF, 1:ON)
#define MONITOR_FLOW              0    // シリアルモニタでフローを表示(0:OFF, 1:ON)
#define MONITOR_ERR_SERVO         0    // シリアルモニタでサーボエラーを表示(0:OFF, 1:ON)
#define MONITOR_ERR_ALL           0    // 全経路の受信エラー率を表示(0:OFF, 1:ON)
#define MONITOR_SEQ               0    // シリアルモニタでシーケンス番号チェックを表示(0:OFF, 1:ON)
#define MONITOR_PAD               0    // シリアルモニタでリモコンのデータを表示(0:OFF, 1:ON)
#define MONITOR_SUPPRESS_DURATION 8000 // 起動直後のタイムアウトメッセージ抑制時間(単位ms)

// I2C設定, I2Cセンサ関連設定
#define I2C0_SPEED       400000 // I2Cの速度(400kHz推奨)
#define IMUAHRS_INTERVAL 10     // IMU/AHRSのセンサの読み取り間隔(ms)
#define IMUAHRS_STOCK    4      // MPUで移動平均を取る際の元にする時系列データの個数
// #define I2C1_SPEED 100000  // I2Cの速度(100kHz推奨?)
// #define I2C1_MERIMOTE_ADDR 0x58 // MerimoteのI2Cアドレス

// SPI設定
#define SPI0_SPEED 30000000 // SPI通信の速度(30MHz)

// PC接続関連設定
#define SERIAL_PC_BPS     115200 // PCとのシリアル速度(モニタリング表示用)
#define SERIAL_PC_TIMEOUT 2000   // PCとのシリアル接続確立タイムアウト(ms)

// ピンアサイン
#define PIN_ERR_LED        25 // LED用 処理が時間内に収まっていない場合に点灯
#define PIN_EN_L           33 // サーボL系統のENピン
#define PIN_EN_R           4  // サーボR系統のENピン
#define PIN_CHIPSELECT_SD  15 // SDカード用のCSピン
#define PIN_CHIPSELECT_LAN 5  // 有線LAN用のCSピン
#define PIN_RESET_LAN      14 // W5500リセットピン(※ボード裏から半田付けにてフリーピンに配線)
#define PIN_I2C0_SDA       22 // I2CのSDAピン
#define PIN_I2C0_SCL       21 // I2CのSCLピン
#define PIN_LED_BT         26 // Bluetooth接続確認用ピン(点滅はペアリング,点灯でリンク確立)

//-------------------------------------------------------------------------
// サーボ設定
//-------------------------------------------------------------------------

// コマンドサーボの種類
// 00: NOSERVO (マウントなし),            01: PWM_S1 (Single PWM)[WIP]
// 11: PCA9685 (I2C_PCA9685toPWM)[WIP], 21: FTBRSX (FUTABA_RSxTTL)[WIP]
// 31: DXL1 (DYNAMIXEL 1.0)[WIP],       32: DXL2 (DYNAMIXEL 2.0)[WIP]
// 43: KOICS3 (KONDO_ICS 3.5 / 3.6),    44: KOPMX (KONDO_PMX)[WIP]
// 51: JRXBUS (JRPROPO_XBUS)[WIP]
// 61: FTCSTS (FEETECH_STS)[WIP],       62: FTCSCS (FEETECH_SCS)[WIP]
#define MOUNT_SERVO_TYPE_L SERVO_TYPE_KOICS3 // L系統のコマンドサーボの種類 (43)
#define MOUNT_SERVO_TYPE_R SERVO_TYPE_KOICS3 // R系統のコマンドサーボの種類 (43)

// サーボ関連設定
#define SERVO_BAUDRATE_L    1250000 // L系統のICSサーボの通信速度bps
#define SERVO_BAUDRATE_R    1250000 // R系統のICSサーボの通信速度bps
#define SERVO_TIMEOUT_L     2       // L系統のICS返信待ちのタイムアウト時間
#define SERVO_TIMEOUT_R     2       // R系統のICS返信待ちのタイムアウト時間
#define SERVO_LOST_ERR_WAIT 6       // 連続何フレームサーボ信号をロストしたら異常とするか

// 各サーボ系統の最大サーボマウント数
#define IXL_MAX 15 // L系統の最大サーボ数. 標準は15.
#define IXR_MAX 15 // R系統の最大サーボ数. 標準は15.

//-------------------------------------------------------------------------
//  サーボ設定配列の実体定義
//-------------------------------------------------------------------------

enum ServoType {                // サーボプロトコルのタイプ
  NOSERVO = SERVO_TYPE_NONE,    // サーボなし
  PWM_S = SERVO_TYPE_PWM_S,     // Single PWM (WIP)
  PCA9685 = SERVO_TYPE_PCA9685, // I2C_PCA9685 to PWM (WIP)
  FTBRSX = SERVO_TYPE_FTBRSX,   // FUTABA_RSxTTL (WIP)
  DXL1 = SERVO_TYPE_DXL1,       // DYNAMIXEL 1.0 (WIP)
  DXL2 = SERVO_TYPE_DXL2,       // DYNAMIXEL 2.0 (WIP)
  KOICS3 = SERVO_TYPE_KOICS3,   // KONDO_ICS 3.5 / 3.6
  KOPMX = SERVO_TYPE_KOPMX,     // KONDO_PMX (WIP)
  JRXBUS = SERVO_TYPE_JRXBUS,   // JRPROPO_XBUS (WIP)
  FTCSTS = SERVO_TYPE_FTCSTS,   // FEETECH_STS (WIP)
  FTCSCS = SERVO_TYPE_FTCSCS    // FEETECH_SCS (WIP)
};

enum ImuAhrsType {               // 6軸9軸センサ種の列挙型(NO_IMU, MPU6050_IMU, MPU9250_IMU, BNO055_AHRS)
  NO_IMU = IMUAHRS_NONE,         // IMU/AHRS なし.
  MPU6050_IMU = IMUAHRS_MPU6050, // MPU6050
  MPU9250_IMU = IMUAHRS_MPU9250, // MPU9250(未設定)
  BNO055_AHRS = IMUAHRS_BNO055   // BNO055
};
// L系統のサーボのマウントの設定
// 00: NOSERVO (マウントなし),            01: PWM_S1 (Single PWM)[WIP]
// 11: PCA9685 (I2C_PCA9685toPWM)[WIP], 21: FTBRSX (FUTABA_RSxTTL)[WIP]
// 31: DXL1 (DYNAMIXEL 1.0)[WIP],       32: DXL2 (DYNAMIXEL 2.0)[WIP]
// 43: KOICS3 (KONDO_ICS 3.5 / 3.6),    44: KOPMX (KONDO_PMX)[WIP]
// 51: JRXBUS (JRPROPO_XBUS)[WIP]
// 61: FTCSTS (FEETECH_STS)[WIP],       62: FTCSCS (FEETECH_SCS)[WIP]
static int IXL_MT[IXL_MAX] = {
    SERVO_TYPE_KOICS3, // [00]頭ヨー
    SERVO_TYPE_NONE,   // [01]左肩ピッチ
    SERVO_TYPE_NONE,   // [02]左肩ロール
    SERVO_TYPE_NONE,   // [03]左肘ヨー
    SERVO_TYPE_NONE,   // [04]左肘ピッチ
    SERVO_TYPE_NONE,   // [05]左股ヨー
    SERVO_TYPE_NONE,   // [06]左股ロール
    SERVO_TYPE_NONE,   // [07]左股ピッチ
    SERVO_TYPE_NONE,   // [08]左膝ピッチ
    SERVO_TYPE_NONE,   // [09]左足首ピッチ
    SERVO_TYPE_NONE,   // [10]左足首ロール
    SERVO_TYPE_NONE,   // [11]追加サーボ用
    SERVO_TYPE_NONE,   // [12]追加サーボ用
    SERVO_TYPE_NONE,   // [13]追加サーボ用
    SERVO_TYPE_NONE    // [14]追加サーボ用
};

// R系統のサーボのマウントの設定
// 00: NOSERVO (マウントなし),            01: PWM_S1 (Single PWM)[WIP]
// 11: PCA9685 (I2C_PCA9685toPWM)[WIP], 21: FTBRSX (FUTABA_RSxTTL)[WIP]
// 31: DXL1 (DYNAMIXEL 1.0)[WIP],       32: DXL2 (DYNAMIXEL 2.0)[WIP]
// 43: KOICS3 (KONDO_ICS 3.5 / 3.6),    44: KOPMX (KONDO_PMX)[WIP]
// 51: JRXBUS (JRPROPO_XBUS)[WIP]
// 61: FTCSTS (FEETECH_STS)[WIP],       62: FTCSCS (FEETECH_SCS)[WIP]
static int IXR_MT[IXR_MAX] = {
    SERVO_TYPE_NONE, // [00]腰ヨー
    SERVO_TYPE_NONE, // [01]右肩ピッチ
    SERVO_TYPE_NONE, // [02]右肩ロール
    SERVO_TYPE_NONE, // [03]右肘ヨー
    SERVO_TYPE_NONE, // [04]右肘ピッチ
    SERVO_TYPE_NONE, // [05]右股ヨー
    SERVO_TYPE_NONE, // [06]右股ロール
    SERVO_TYPE_NONE, // [07]右股ピッチ
    SERVO_TYPE_NONE, // [08]右膝ピッチ
    SERVO_TYPE_NONE, // [09]右足首ピッチ
    SERVO_TYPE_NONE, // [10]右足首ロール
    SERVO_TYPE_NONE, // [11]追加サーボ用
    SERVO_TYPE_NONE, // [12]追加サーボ用
    SERVO_TYPE_NONE, // [13]追加サーボ用
    SERVO_TYPE_NONE  // [14]追加サーボ用
};

// L系統のコード上のサーボIndexに対し, 実際に呼び出すハードウェアのID番号
static int IXL_ID[IXL_MAX] = {
    0,  // [00]頭ヨー
    1,  // [01]左肩ピッチ
    2,  // [02]左肩ロール
    3,  // [03]左肘ヨー
    4,  // [04]左肘ピッチ
    5,  // [05]左股ヨー
    6,  // [06]左股ロール
    7,  // [07]左股ピッチ
    8,  // [08]左膝ピッチ
    9,  // [09]左足首ピッチ
    10, // [10]左足首ロール
    11, // [11]追加サーボ用
    12, // [12]追加サーボ用
    13, // [13]追加サーボ用
    14  // [14]追加サーボ用
};

// R系統のコード上のサーボIndexに対し, 実際に呼び出すハードウェアのID番号
static int IXR_ID[IXR_MAX] = {
    0,  // [00]腰ヨー
    1,  // [01]右肩ピッチ
    2,  // [02]右肩ロール
    3,  // [03]右肘ヨー
    4,  // [04]右肘ピッチ
    5,  // [05]右股ヨー
    6,  // [06]右股ロール
    7,  // [07]右股ピッチ
    8,  // [08]右膝ピッチ
    9,  // [09]右足首ピッチ
    10, // [10]右足首ロール
    11, // [11]追加サーボ用
    12, // [12]追加サーボ用
    13, // [13]追加サーボ用
    14  // [14]追加サーボ用
};

// L系統のサーボ回転方向補正(1:変更なし, -1:逆転)
static int IXL_CW[IXL_MAX] = {
    1, // [00]頭ヨー
    1, // [01]左肩ピッチ
    1, // [02]左肩ロール
    1, // [03]左肘ヨー
    1, // [04]左肘ピッチ
    1, // [05]左股ヨー
    1, // [06]左股ロール
    1, // [07]左股ピッチ
    1, // [08]左膝ピッチ
    1, // [09]左足首ピッチ
    1, // [10]左足首ロール
    1, // [11]追加サーボ用
    1, // [12]追加サーボ用
    1, // [13]追加サーボ用
    1  // [14]追加サーボ用
};

// R系統のサーボ回転方向補正(1:変更なし, -1:逆転)
static int IXR_CW[IXR_MAX] = {
    1, // [00]腰ヨー
    1, // [01]右肩ピッチ
    1, // [02]右肩ロール
    1, // [03]右肘ヨー
    1, // [04]右肘ピッチ
    1, // [05]右股ヨー
    1, // [06]右股ロール
    1, // [07]右股ピッチ
    1, // [08]右膝ピッチ
    1, // [09]右足首ピッチ
    1, // [10]右足首ロール
    1, // [11]追加サーボ用
    1, // [12]追加サーボ用
    1, // [13]追加サーボ用
    1  // [14]追加サーボ用
};

// L系統のトリム値(degree)
static float IXL_TRIM[IXL_MAX] = {
    0.0,     // [00]頭ヨー
    -20.42,  // [01]左肩ピッチ
    -103.55, // [02]左肩ロール
    0.85,    // [03]左肘ヨー
    88.41,   // [04]左肘ピッチ
    0.0,     // [05]左股ヨー
    -2.0,    // [06]左股ロール
    -17.12,  // [07]左股ピッチ
    -68.54,  // [08]左膝ピッチ
    -25.7,   // [09]左足首ピッチ
    0.0,     // [10]左足首ロール
    0.0,     // [11]追加サーボ用
    0.0,     // [12]追加サーボ用
    0.0,     // [13]追加サーボ用
    0.0      // [14]追加サーボ用
};

// R系統のトリム値(degree)
static float IXR_TRIM[IXR_MAX] = {
    -4.28,  // [00]腰ヨー
    0.68,   // [01]右肩ピッチ
    -89.41, // [02]右肩ロール
    0.0,    // [03]右肘ヨー
    91.83,  // [04]右肘ピッチ
    0.0,    // [05]右股ヨー
    1.0,    // [06]右股ロール
    -21.42, // [07]右股ピッチ
    -53.68, // [08]右膝ピッチ
    -24.12, // [09]右足首ピッチ
    0.0,    // [10]右足首ロール
    0.0,    // [11]追加サーボ用
    0.0,    // [12]追加サーボ用
    0.0,    // [13]追加サーボ用
    0.0     // [14]追加サーボ用
};

//------------------------------------------------------------------------------------
// PADの種類定義
//------------------------------------------------------------------------------------

enum PadType {
  // リモコン種の列挙型(NONE, PC, MERIMOTE, BLUERETRO, SBDBT, KRR5FH)
  PAD_NONE = PAD_TYPE_NONE,       // リモコンなし
  PC = PAD_TYPE_PC,               // PCからのPD入力情報を使用
  MERIMOTE = PAD_TYPE_MERIMOTE,   // MERIMOTE(未導入)
  BLUERETRO = PAD_TYPE_BLUERETRO, // BLUERETRO(未導入)
  SBDBT = PAD_TYPE_SBDBT,         // SBDBT(未導入)
  KRR5FH = PAD_TYPE_KRR5FH,       // KRR5FH
  WIIMOTE = PAD_TYPE_WIIMOTE,     // WIIMOTE / WIIMOTE + Nunchuk
  WIIMOTE_C = PAD_TYPE_WIIMOTE_C, // WIIMOTE+Classic
};
// JOYPAD関連設定
#define PAD_INIT_TIMEOUT 10000         // 起動時のJOYPADの接続確立のタイムアウト(ms)
#define PAD_INTERVAL     10            // JOYPADのデータを読みに行くフレーム間隔 (※KRC-5FHでは4推奨)
#define PAD_BUTTON_MARGE 1             // 0:JOYPADのボタンデータをMeridim受信値に論理積, 1:Meridim受信値に論理和
#define PAD_GENERALIZE   1             // ジョイパッドの入力値をPS系に一般化する
#define PAD_MOUNT        PAD_TYPE_NONE // // ジョイパッドの搭載 PC, MERIMOTE, BLUERETRO, KRR5FH, WIIMOTE

//------------------------------------------------------------------------------------
// システム用
//------------------------------------------------------------------------------------
const int MRDM_BYTE = MRDM_LEN * 2;    // Meridim配列のバイト型の長さ
const int MRD_ERR = MRDM_LEN - 2;      // エラーフラグの格納場所(配列の末尾から2つめ)
const int MRD_ERR_u = MRD_ERR * 2 + 1; // エラーフラグの格納場所(上位8ビット)
const int MRD_ERR_l = MRD_ERR * 2;     // エラーフラグの格納場所(下位8ビット)
const int MRD_CKSM = MRDM_LEN - 1;     // チェックサムの格納場所(配列の末尾)
const int PAD_LEN = 5;                 // リモコン用配列の長さ

#endif // __MERIDIAN_CONFIG__
