#ifndef __MERIDIAN_CONFIG__
#define __MERIDIAN_CONFIG__

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
//  サーボIDとロボット部位、軸との対応表 (KHR-3HVの例)
//==================================================================================================
//
// ID    Parts/Axis ＜ICS_Left_Upper SIO1,SIO2＞
// [L00] 頭/ヨー
// [L01] 左肩/ピッチ
// [L02] 左肩/ロール
// [L03] 左肘/ヨー
// [L04] 左肘/ピッチ
// [L05] -
// ID    Parts/Axis ＜ICS_Left_Lower SIO3,SIO4＞
// [L06] 左股/ロール
// [L07] 左股/ピッチ
// [L08] 左膝/ピッチ
// [L09] 左足首/ピッチ
// [L10] 左足首/ロール
// ID    Parts/Axis  ＜ICS_Right_Upper SIO5,SIO6＞
// [R00] 腰/ヨー
// [R01] 右肩/ピッチ
// [R02] 右肩/ロール
// [R03] 右肘/ヨー
// [R04] 右肘/ピッチ
// [R05] -
// ID    Parts/Axis  ＜ICS_Right_Lower SIO7,SIO8＞
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
// [02]-[04] IMU/AHRS:acc＿x,acc＿y,acc＿z    加速度x,y,z
// [05]-[07] IMU/AHRS:gyro＿x,gyro＿y,gyro＿z ジャイロx,y,z
// [08]-[10] IMU/AHRS:mag＿x,mag＿y,mag＿z    磁気コンパスx,y,z
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
#define MOUNT_SD      1           // SDカードリーダーの有無s(0:なし, 1:あり)
#define MOUNT_IMUAHRS BNO055_AHRS // IMU/AHRSの搭載 NO_IMU, MPU6050_IMU, MPU9250_IMU, BNO055_AHRS
#define MOUNT_PAD     PC          // ジョイパッドの搭載 PC, MERIMOTE, BLUERETRO, KRR5FH, WIIMOTE

// 動作モード
#define MODE_ESP32_STANDALONE 0 // ESP32をボードに挿さず動作確認(0:NO, 1:YES)
#define MODE_UDP_RECEIVE      1 // PCからのデータ受信(0:OFF, 1:ON, 通常は1)
#define MODE_UDP_SEND         1 // PCへのデータ送信(0:OFF, 1:ON, 通常は1)

// Wifiの設定(SSID,パスワード等は別途keys.hで指定)
#define MODE_FIXED_IP 0 // IPアドレスを固定するか(0:NO, 1:YES)
#define UDP_TIMEOUT   4 // UDPの待受タイムアウト(単位ms,推奨値0)

// EEPROMの設定
#define EEPROM_SIZE    540 // 使用するEEPROMのサイズ(バイト)
#define EEPROM_SET     1   // 起動時にEEPROMにconfig.hの内容をセット(mrd_set_eeprom)
#define EEPROM_PROTECT 0   // EEPROMの書き込み保護(0:保護しない, 1:書き込み禁止)
#define EEPROM_LOAD    1   // 起動時にEEPROMの内容を諸設定にロードする
#define EEPROM_DUMP    1   // 起動時のEEPROM内容のダンプ表示
#define EEPROM_STYLE   Dec // 起動時のEEPROM内容のダンプ表示の書式(Bin,Hex,Dec)

// 動作チェックモード
#define CHECK_SD_RW     1 // 起動時のSDカードリーダーの読み書きチェック
#define CHECK_EEPROM_RW 0 // 起動時のEEPROMの動作チェック

// シリアルモニタリング
#define MONITOR_FRAME_DELAY       1    // シリアルモニタでフレーム遅延時間を表示(0:OFF, 1:ON)
#define MONITOR_FLOW              0    // シリアルモニタでフローを表示(0:OFF, 1:ON)
#define MONITOR_ERR_SERVO         0    // シリアルモニタでサーボエラーを表示(0:OFF, 1:ON)
#define MONITOR_ERR_ALL           0    // 全経路の受信エラー率を表示
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
#define SPI0_SPEED 6000000 // SPI通信の速度(6000000kHz推奨)

// PC接続関連設定
#define SERIAL_PC_BPS     115200 // PCとのシリアル速度(モニタリング表示用)
#define SERIAL_PC_TIMEOUT 2000   // PCとのシリアル接続確立タイムアウト(ms)

// JOYPAD関連設定
#define PAD_INIT_TIMEOUT 10000 // 起動時のJOYPADの接続確立のタイムアウト(ms)
#define PAD_INTERVAL     10    // JOYPADのデータを読みに行くフレーム間隔 (※KRC-5FHでは4推奨)
#define PAD_BUTTON_MARGE 1     // 0:JOYPADのボタンデータをMeridim受信値に論理積, 1:Meridim受信値に論理和
#define PAD_GENERALIZE   1     // ジョイパッドの入力値をPS系に一般化する

// ピンアサイン
#define PIN_ERR_LED       25 // LED用 処理が時間内に収まっていない場合に点灯
#define PIN_EN_L          33 // サーボL系統のENピン
#define PIN_EN_R          4  // サーボR系統のENピン
#define PIN_CHIPSELECT_SD 15 // SDカード用のCSピン
#define PIN_I2C0_SDA      22 // I2CのSDAピン
#define PIN_I2C0_SCL      21 // I2CのSCLピン
#define PIN_LED_BT        26 // Bluetooth接続確認用ピン(点滅はペアリング,点灯でリンク確立)

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
#define MOUNT_SERVO_TYPE_L 43 // L系統のコマンドサーボの種類
#define MOUNT_SERVO_TYPE_R 43 // R系統のコマンドサーボの種類

// サーボ関連設定
#define SERVO_BAUDRATE_L    1250000 // L系統のICSサーボの通信速度bps
#define SERVO_BAUDRATE_R    1250000 // R系統のICSサーボの通信速度bps
#define SERVO_TIMEOUT_L     2       // L系統のICS返信待ちのタイムアウト時間
#define SERVO_TIMEOUT_R     2       // R系統のICS返信待ちのタイムアウト時間
#define SERVO_LOST_ERR_WAIT 6       // 連続何フレームサーボ信号をロストしたら異常とするか

// 各サーボ系統の最大サーボマウント数
#define IXL_MAX 15 // L系統の最大サーボ数. 標準は15.
#define IXR_MAX 15 // R系統の最大サーボ数. 標準は15.

// L系統のサーボのマウントの設定
// 00: NOSERVO (マウントなし),            01: PWM_S1 (Single PWM)[WIP]
// 11: PCA9685 (I2C_PCA9685toPWM)[WIP], 21: FTBRSX (FUTABA_RSxTTL)[WIP]
// 31: DXL1 (DYNAMIXEL 1.0)[WIP],       32: DXL2 (DYNAMIXEL 2.0)[WIP]
// 43: KOICS3 (KONDO_ICS 3.5 / 3.6),    44: KOPMX (KONDO_PMX)[WIP]
// 51: JRXBUS (JRPROPO_XBUS)[WIP]
// 61: FTCSTS (FEETECH_STS)[WIP],       62: FTCSCS (FEETECH_SCS)[WIP]
int IXL_MT[IXL_MAX] = {
    // L系統のマウント状態
    43, // [00]頭ヨー
    43, // [01]左肩ピッチ
    43, // [02]左肩ロール
    43, // [03]左肘ヨー
    43, // [04]左肘ピッチ
    43, // [05]左股ヨー
    43, // [06]左股ロール
    43, // [07]左股ピッチ
    43, // [08]左膝ピッチ
    43, // [09]左足首ピッチ
    43, // [10]左足首ロール
    0,  // [11]追加サーボ用
    0,  // [12]追加サーボ用
    0,  // [13]追加サーボ用
    0   // [14]追加サーボ用
};

// R系統のサーボのマウントの設定
// 00: NOSERVO (マウントなし),            01: PWM_S1 (Single PWM)[WIP]
// 11: PCA9685 (I2C_PCA9685toPWM)[WIP], 21: FTBRSX (FUTABA_RSxTTL)[WIP]
// 31: DXL1 (DYNAMIXEL 1.0)[WIP],       32: DXL2 (DYNAMIXEL 2.0)[WIP]
// 43: KOICS3 (KONDO_ICS 3.5 / 3.6),    44: KOPMX (KONDO_PMX)[WIP]
// 51: JRXBUS (JRPROPO_XBUS)[WIP]
// 61: FTCSTS (FEETECH_STS)[WIP],       62: FTCSCS (FEETECH_SCS)[WIP]
int IXR_MT[IXR_MAX] = {
    // R系統のマウント状態
    43, // [00]腰ヨー
    43, // [01]右肩ピッチ
    43, // [02]右肩ロール
    43, // [03]右肘ヨー
    43, // [04]右肘ピッチ
    43, // [05]右股ヨー
    43, // [06]右股ロール
    43, // [07]右股ピッチ
    43, // [08]右膝ピッチ
    43, // [09]右足首ピッチ
    43, // [10]右足首ロール
    0,  // [11]追加サーボ用
    0,  // [12]追加サーボ用
    0,  // [13]追加サーボ用
    0   // [14]追加サーボ用
};

// L系統のコード上のサーボIndexに対し, 実際に呼び出すハードウェアのID番号
int IXL_ID[IXL_MAX] = {
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
int IXR_ID[IXR_MAX] = {
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
int IXL_CW[IXL_MAX] = {
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
int IXR_CW[IXR_MAX] = {
    // R系統の正転逆転
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
float IDL_TRIM[IXL_MAX] = {
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
    0.0,     // [14]追加サーボ用
};

// R系統のトリム値(degree)
float IDR_TRIM[IXR_MAX] = {
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
    0.0,    // [14]追加サーボ用
};

//-------------------------------------------------------------------------
//  固定値, マスターコマンド定義
//-------------------------------------------------------------------------
// 固定値, マスターコマンド定義
#define MCMD_TORQUE_ALL_OFF         0      // すべてのサーボトルクをオフにする(脱力)
#define MCMD_DUMMY_DATA             -32768 // SPI送受信用のダミーデータ判定用
#define MCMD_TEST_VALUE             -32767 // テスト用の仮設変数
#define MCMD_SENSOR_YAW_CALIB       10002  // センサの推定ヨー軸を現在値センターとしてリセット
#define MCMD_SENSOR_ALL_CALIB       10003  // センサの3軸について現在値を原点としてリセット
#define MCMD_ERR_CLEAR_SERVO_ID     10004  // 通信エラーのサーボのIDをクリア(MRD_ERR_l)
#define MCMD_BOARD_TRANSMIT_ACTIVE  10005  // ボードが定刻で送信を行うモード(PC側が受信待ち)
#define MCMD_BOARD_TRANSMIT_PASSIVE 10006  // ボードが受信を待ち返信するモード(PC側が定刻送信)
#define MCMD_FRAMETIMER_RESET       10007  // フレームタイマーを現在時刻にリセット
#define MCMD_BOARD_STOP_DURING      10008  // ボードの末端処理を[MRD_STOP_FRAMES]ミリ秒止める
#define MCMD_EEPROM_ENTER_WRITE     10009  // EEPROM書き込みモードのスタート
#define MCMD_EEPROM_EXIT_WRITE      10010  // EEPROM書き込みモードの終了
#define MCMD_EEPROM_ENTER_READ      10011  // EEPROM読み出しモードのスタート
#define MCMD_EEPROM_EXIT_READ       10012  // EEPROM読み出しモードの終了
#define MCMD_SDCARD_ENTER_WRITE     10013  // SDCARD書き込みモードのスタート
#define MCMD_SDCARD_EXIT_WRITE      10014  // SDCARD書き込みモードの終了
#define MCMD_SDCARD_ENTER_READ      10015  // SDCARD読み出しモードのスタート
#define MCMD_SDCARD_EXIT_READ       10016  // SDCARD読み出しモードの終了
#define MCMD_START_TRIM_SETTING     10100  // トリム設定モードに入る(Meridian_console.py連携)
#define MCMD_EEPROM_SAVE_TRIM       10101  // 現在の姿勢をトリム値としてEEPROMに書き込む
#define MCMD_EEPROM_LOAD_TRIM       10102  // EEPROMのトリム値をサーボに反映する
#define MCMD_EEPROM_BOARDTOPC_DATA0 10200  // EEPROMの[0][*]をボードからPCにMeridimで送信する
#define MCMD_EEPROM_BOARDTOPC_DATA1 10201  // EEPROMの[1][*]をボードからPCにMeridimで送信する
#define MCMD_EEPROM_BOARDTOPC_DATA2 10202  // EEPROMの[2][*]をボードからPCにMeridimで送信する
#define MCMD_EEPROM_PCTOBOARD_DATA0 10300  // EEPROM用の[0][*]をPCからボードにMeridimで送信する
#define MCMD_EEPROM_PCTOBOARD_DATA1 10301  // EEPROM用の[1][*]をPCからボードにMeridimで送信する
#define MCMD_EEPROM_PCTOBOARD_DATA2 10302  // EEPROM用の[2][*]をPCからボードにMeridimで送信する

#define MCMD_NAK 32766 // コマンド実行の失敗を応答
#define MCMD_ACK 32767 // コマンド実行の成功を応答

//-------------------------------------------------------------------------
//  Meridim90 配列アクセス対応キー
//-------------------------------------------------------------------------
#define MRD_MASTER        0  // マスターコマンド
#define MRD_SEQ           1  // シーケンス番号
#define MRD_ACC_X         2  // 加速度センサX値
#define MRD_ACC_Y         3  // 加速度センサY値
#define MRD_ACC_Z         4  // 加速度センサZ値
#define MRD_GYRO_X        5  // ジャイロセンサX値
#define MRD_GYRO_Y        6  // ジャイロセンサY値
#define MRD_GYRO_Z        7  // ジャイロセンサZ値
#define MRD_MAG_X         8  // 磁気コンパスX値
#define MRD_MAG_Y         9  // 磁気コンパスY値
#define MRD_MAG_Z         10 // 磁気コンパスZ値
#define MRD_TEMP          11 // 温度センサ値
#define MRD_DIR_ROLL      12 // DMP推定ロール方向値
#define MRD_DIR_PITCH     13 // DMP推定ピッチ方向値
#define MRD_DIR_YAW       14 // DMP推定ヨー方向値
#define MRD_PAD_BUTTONS   15 // リモコンの基本ボタン値
#define MRD_PAD_STICK_L   16 // リモコンの左スティックアナログ値
#define MRD_PAD_STICK_R   17 // リモコンの右スティックアナログ値
#define MRD_PAD_L2R2VAL   18 // リモコンのL2R2ボタンアナログ値
#define MRD_MOTION_FRAMES 19 // モーション設定のフレーム数
#define MRD_STOP_FRAMES   19 // ボード停止時のフレーム数(MCMD_BOARD_STOP_DURINGで指定)
#define C_HEAD_Y_CMD      20 // 頭ヨーのコマンド
#define C_HEAD_Y_VAL      21 // 頭ヨーの値
#define L_SHOULDER_P_CMD  22 // 左肩ピッチのコマンド
#define L_SHOULDER_P_VAL  23 // 左肩ピッチの値
#define L_SHOULDER_R_CMD  24 // 左肩ロールのコマンド
#define L_SHOULDER_R_VAL  25 // 左肩ロールの値
#define L_ELBOW_Y_CMD     26 // 左肘ヨーのコマンド
#define L_ELBOW_Y_VAL     27 // 左肘ヨーの値
#define L_ELBOW_P_CMD     28 // 左肘ピッチのコマンド
#define L_ELBOW_P_VAL     29 // 左肘ピッチの値
#define L_HIPJOINT_Y_CMD  30 // 左股ヨーのコマンド
#define L_HIPJOINT_Y_VAL  31 // 左股ヨーの値
#define L_HIPJOINT_R_CMD  32 // 左股ロールのコマンド
#define L_HIPJOINT_R_VAL  33 // 左股ロールの値
#define L_HIPJOINT_P_CMD  34 // 左股ピッチのコマンド
#define L_HIPJOINT_P_VAL  35 // 左股ピッチの値
#define L_KNEE_P_CMD      36 // 左膝ピッチのコマンド
#define L_KNEE_P_VAL      37 // 左膝ピッチの値
#define L_ANKLE_P_CMD     38 // 左足首ピッチのコマンド
#define L_ANKLE_P_VAL     39 // 左足首ピッチの値
#define L_ANKLE_R_CMD     40 // 左足首ロールのコマンド
#define L_ANKLE_R_VAL     41 // 左足首ロールの値
#define L_SERVO_IX11_CMD  42 // 追加サーボ用のコマンド
#define L_SERVO_IX11_VAL  43 // 追加サーボ用の値
#define L_SERVO_IX12_CMD  44 // 追加サーボ用のコマンド
#define L_SERVO_IX12_VAL  45 // 追加サーボ用の値
#define L_SERVO_IX13_CMD  46 // 追加サーボ用のコマンド
#define L_SERVO_IX13_VAL  47 // 追加サーボ用の値
#define L_SERVO_IX14_CMD  48 // 追加サーボ用のコマンド
#define L_SERVO_IX14_VAL  49 // 追加サーボ用の値

#define C_WAIST_Y_CMD    50 // 腰ヨーのコマンド
#define C_WAIST_Y_VAL    51 // 腰ヨーの値
#define R_SHOULDER_P_CMD 52 // 右肩ピッチのコマンド
#define R_SHOULDER_P_VAL 53 // 右肩ピッチの値
#define R_SHOULDER_R_CMD 54 // 右肩ロールのコマンド
#define R_SHOULDER_R_VAL 55 // 右肩ロールの値
#define R_ELBOW_Y_CMD    56 // 右肘ヨーのコマンド
#define R_ELBOW_Y_VAL    57 // 右肘ヨーの値
#define R_ELBOW_P_CMD    58 // 右肘ピッチのコマンド
#define R_ELBOW_P_VAL    59 // 右肘ピッチの値
#define R_HIPJOINT_Y_CMD 60 // 右股ヨーのコマンド
#define R_HIPJOINT_Y_VAL 61 // 右股ヨーの値
#define R_HIPJOINT_R_CMD 62 // 右股ロールのコマンド
#define R_HIPJOINT_R_VAL 63 // 右股ロールの値
#define R_HIPJOINT_P_CMD 64 // 右股ピッチのコマンド
#define R_HIPJOINT_P_VAL 65 // 右股ピッチの値
#define R_KNEE_P_CMD     66 // 右膝ピッチのコマンド
#define R_KNEE_P_VAL     67 // 右膝ピッチの値
#define R_ANKLE_P_CMD    68 // 右足首ピッチのコマンド
#define R_ANKLE_P_VAL    69 // 右足首ピッチの値
#define R_ANKLE_R_CMD    70 // 右足首ロールのコマンド
#define R_ANKLE_R_VAL    71 // 右足首ロールの値
#define R_SERVO_IX11_CMD 72 // 追加テスト用のコマンド
#define R_SERVO_IX11_VAL 73 // 追加テスト用の値
#define R_SERVO_IX12_CMD 74 // 追加テスト用のコマンド
#define R_SERVO_IX12_VAL 75 // 追加テスト用の値
#define R_SERVO_IX13_CMD 76 // 追加テスト用のコマンド
#define R_SERVO_IX13_VAL 77 // 追加テスト用の値
#define R_SERVO_IX14_CMD 78 // 追加テスト用のコマンド
#define R_SERVO_IX14_VAL 79 // 追加テスト用の値

#define MRD_USERDATA_80 80 // ユーザー定義用
#define MRD_USERDATA_81 81 // ユーザー定義用
#define MRD_USERDATA_82 82 // ユーザー定義用
#define MRD_USERDATA_83 83 // ユーザー定義用
#define MRD_USERDATA_84 84 // ユーザー定義用
#define MRD_USERDATA_85 85 // ユーザー定義用
#define MRD_USERDATA_86 86 // ユーザー定義用
#define MRD_USERDATA_87 87 // ユーザー定義用
// #define MRD_ERR         88 // エラーコード (MRDM_LEN - 2)
// #define MRD_CKSM        89 // チェックサム (MRDM_LEN - 1)

// エラービット MRD_ERR_CODEの上位8bit分
#define ERRBIT_15_ESP_PC       15 // ESP32 → PC のUDP受信エラー (0:エラーなし、1:エラー検出)
#define ERRBIT_14_PC_ESP       14 // PC → ESP32 のUDP受信エラー
#define ERRBIT_13_ESP_TSY      13 // ESP32 → TeensyのSPI受信エラー
#define ERRBIT_12_TSY_ESP      12 // Teensy → ESP32 のSPI受信エラー
#define ERRBIT_11_BOARD_DELAY  11 // Teensy or ESP32の処理ディレイ (末端で捕捉)
#define ERRBIT_10_UDP_ESP_SKIP 10 // PC → ESP32 のUDPフレームスキップエラー
#define ERRBIT_9_BOARD_SKIP    9  // PC → ESP32 → Teensy のフレームスキップエラー(末端で捕捉)
#define ERRBIT_8_PC_SKIP       8  // Teensy → ESP32 → PC のフレームスキップエラー(末端で捕捉)

#endif // __MERIDIAN_CONFIG__
