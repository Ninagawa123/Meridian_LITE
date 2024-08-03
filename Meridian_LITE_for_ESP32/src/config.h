#ifndef __MERIDIAN_CONFIG__
#define __MERIDIAN_CONFIG__

//================================================================================================================
//  MERIDIAN - LITE - ESP32の配線
//================================================================================================================
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
//   27          -  SPI_MISO
//   23          -  SPI_MOSI
//   18          -  SPI_CSK
//   15          -  SPI_CS SD

//================================================================================================================
//  サーボIDとロボット部位、軸との対応表 (KHR-3HVの例)
//================================================================================================================
//
// ID    Parts/Axis　＜ICS_Left_Upper SIO1,SIO2＞
// [L00] 頭/ヨー
// [L01] 左肩/ピッチ
// [L02] 左肩/ロール
// [L03] 左肘/ヨー
// [L04] 左肘/ピッチ
// [L05] -
// ID    Parts/Axis　＜ICS_Left_Lower SIO3,SIO4＞
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

//================================================================================================================
//  Meridim90配列 一覧表
//================================================================================================================
//
// [00]      マスターコマンド デフォルトは90 で配列数も同時に示す
// [01]      シーケンス番号
// [02]-[04] IMU/AHRS:acc＿x,acc＿y,acc＿z    加速度x,y,z
// [05]-[07] IMU/AHRS:gyro＿x,gyro＿y,gyro＿z ジャイロx,y,z
// [08]-[10] IMU/AHRS:mag＿x,mag＿y,mag＿z    磁気コンパスx,y,z
// [11]      IMU/AHRS:temp                   温度
// [12]-[14] IMU/AHRS:DMP ROLL,PITCH,YAW     DMP推定値 ロール,ピッチ,ヨー
// [15]      ボタンデータ1
// [16]      ボタンアナログ1（Stick Left）
// [17]      ボタンアナログ2（Stick Right）
// [18]      ボタンアナログ3（L2,R2 アナログ）
// [19]      モーション設定（フレーム数）
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

// Meridimの基本設定
#define MRDM_LEN       90  // Meridim配列の長さ設定（デフォルトは90）
#define FRAME_DURATION 10  // 1フレームあたりの単位時間（単位ms）
#define CHARGE_TIME    200 // 起動時のコンデンサチャージ待機時間（単位ms）

// 動作モード
#define MODE_ESP32_STDALONE 0 // ESP32をボードに挿さず動作確認（0:NO, 1:YES）
// （↑サーボを無視し、L0番サーボ値として+-30度のサインカーブを代入）
#define MODE_UDP_RECEIVE 1 // PCからのデータ受信（0:OFF, 1:ON, 通常は1）
#define MODE_UDP_SEND    1 // PCへのデータ送信（0:OFF, 1:ON, 通常は1）

// Wifiの設定(SSID,パスワード等は別途keys.hで指定)
#define MODE_FIXED_IP 0 // IPアドレスを固定するか（0:NO, 1:YES）
#define UDP_TIMEOUT   4 // UDPの待受タイムアウト（単位ms,推奨値0）

// EEPROMの設定
#define EEPROM_SIZE    540 // 使用するEEPROMのサイズ(バイト)
#define EEPROM_SET     0   // 起動時にEEPROMにconfig.hの内容をセット(mrd_set_eeprom)
#define EEPROM_PROTECT 0   // EEPROMの書き込み保護(0:保護しない, 1:書き込み禁止)
#define EEPROM_LOAD    0   // 起動時にEEPROMの内容を諸設定にロードする(未導入)
#define EEPROM_DUMP    0   // 起動時のEEPROM内容のダンプ表示
#define EEPROM_FORMAT  Dec // 起動時のEEPROM内容のダンプ表示の書式(Bin,Hex,Dec)

// 動作チェックモード
#define CHECK_SD_RW     0 // 起動時のSDカードリーダーの読み書きチェック
#define CHECK_EEPROM_RW 0 // 起動時のEEPROMの動作チェック

// シリアルモニタリング
#define MONITOR_FLOW       0 // シリアルモニタでフローを表示（0:OFF, 1:ON）
#define MONITOR_ALL_ERROR  0 // 全経路の受信エラー率を表示
#define MONITOR_SERVO_ERR  0 // シリアルモニタでサーボエラーを表示（0:OFF, 1:ON）
#define MONITOR_SEQ_NUMBER 0 // シリアルモニタでシーケンス番号チェックを表示（0:OFF, 1:ON）
#define MONITOR_PAD        0 // シリアルモニタでリモコンのデータを表示（0:OFF, 1:ON）

// 各種ハードウェアのマウント有無
#define MOUNT_SD 0 // SDカードリーダーの有無 (0:なし, 1:あり)

// I2C設定, I2Cセンサ関連設定
#define MOUNT_IMUAHRS    BNO055_AHRS // IMU/AHRSの搭載 NO_IMU, MPU6050_IMU, MPU9250_IMU, BNO055_AHRS
#define I2C0_SPEED       400000      // I2Cの速度（400kHz推奨）
#define IMUAHRS_STOCK    4 // MPUで移動平均を取る際の元にする時系列データの個数
#define IMUAHRS_INTERVAL 10 // IMU/AHRSのセンサの読み取り間隔(ms)
// #define I2C1_SPEED 100000  // I2Cの速度（100kHz推奨?）
// #define I2C1_MERIMOTE_ADDR 0x58 // MerimoteのI2Cアドレス

// SPI設定
#define SPI0_SPEED 6000000 // SPI通信の速度（6000000kHz推奨）

// JOYPAD関連設定
#define MOUNT_PAD        KRR5FH // ジョイパッドの搭載 PC, MERIMOTE, BLUERETRO, SBDBT, KRR5FH
#define PAD_INTERVAL     10 // JOYPADのデータを読みに行くフレーム間隔 (※KRC-5FHでは4推奨)
#define PAD_BUTTON_MARGE 1 // 0:JOYPADのボタンデータをMeridim受信値に論理積, 1:Meridim受信値に論理和
#define PAD_GENERALIZE   1 // ジョイパッドの入力値をPS系に一般化する

// コマンドサーボの種類
// 00: マウントなし
// 01: Single PWM (Not yet.)
// 11: I2C_PCA9685 to PWM (Not yet.)
// 21: FUTABA_RSxTTL (Not yet.)
// 31: DYNAMIXEL Protocol 1.0 (Not yet.)
// 32: DYNAMIXEL Protocol 2.0 (Not yet.)
// 43: KONDO_ICS 3.5/3.6
// 44: KONDO_PMX (Not yet.)
// 51: JRPROPO_XBUS (Not yet.)
// 61: FEETECH_STS (Not yet.)
// 62: FEETECH_SCS (Not yet.)
#define MOUNT_L_SERVO_TYPE 43 // L系統のコマンドサーボの種類
#define MOUNT_R_SERVO_TYPE 43 // R系統のコマンドサーボの種類

// サーボ関連設定
#define SERVO_BAUDRATE_L      1250000 // L系統のICSサーボの通信速度bps
#define SERVO_BAUDRATE_R      1250000 // R系統のICSサーボの通信速度bps
#define SERVO_TIMEOUT_L       2       // L系統のICS返信待ちのタイムアウト時間
#define SERVO_TIMEOUT_R       2       // R系統のICS返信待ちのタイムアウト時間
#define SERVO_LOST_ERROR_WAIT 6 // 連続何フレームサーボ信号をロストしたら異常とするか

// PC接続関連設定
#define SERIAL_PC_BPS 115200 // PCとのシリアル速度（モニタリング表示用）

// SPI設定
#define SPI_SPEED 6000000 // SPI通信の速度（6000000kHz推奨）

// 固定値, マスターコマンド定義
#define MCMD_UPDATE_YAW_CENTER       10002 // センサの推定ヨー軸を現在値センターとしてリセット
#define MCMD_ENTER_TRIM_MODE         10003 // トリムモードに入る（全サーボオンで垂直に気おつけ姿勢で立つ）
#define MCMD_CLEAR_SERVO_ERROR_ID    10004 // 通信エラーのサーボのIDをクリア(MRD_ERR_l)
#define MCMD_BOARD_TRANSMIT_ACTIVE   10005 // ボードが定刻で送信を行うモード（PC側が待ち受け）
#define MCMD_BOARD_TRANSMIT_PASSIVE  10006 // ボードが受信を待ち返信するモード（PC側が定刻送信）
#define MCMD_RESET_MRD_TIMER         10007 // フレーム管理時計mrd_t_milを現在時刻にリセット
#define MCMD_STOP_BOARD_DURING       10008 // ボードの末端処理をmeridim[MRD_STOP_FRAMES_MS]ミリ秒止める.
#define MCMD_ENTER_EEPROM_WRITE_MODE 10009 // EEPROM書き込みモードのスタート
#define MCMD_EXIT_EEPROM_WRITE_MODE  10010 // EEPROM書き込みモードの終了
#define MCMD_ENTER_EEPROM_READ_MODE  10011 // EEPROM読み出しモードのスタート
#define MCMD_EXIT_EEPROM_READ_MODE   10012 // EEPROM読み出しモードの終了
#define MCMD_ENTER_SDCARD_WRITE_MODE 10013 // SDCARD書き込みモードのスタート
#define MCMD_EXIT_SDCARD_WRITE_MODE  10014 // SDCARD書き込みモードの終了
#define MCMD_ENTER_SDCARD_READ_MODE  10015 // SDCARD読み出しモードのスタート
#define MCMD_EXIT_SDCARD_READ_MODE   10016 // SDCARD読み出しモードの終了
#define MCMD_DUMMY_DATA              -32768 // SPI送受信のダミーデータ判定用(TWIN等で使用)

// ピンアサイン
#define PIN_ERR_LED       25 // LED用 処理が時間内に収まっていない場合に点灯
#define PIN_EN_L          33 // サーボL系統のENピン
#define PIN_EN_R          4  // サーボR系統のENピン
#define PIN_CHIPSELECT_SD 15 // SDカード用のCSピン
#define PIN_I2C0_SDA      22 // I2CのSDAピン
#define PIN_I2C0_SCL      21 // I2CのSCLピン

//-------------------------------------------------------------------------
//  サ ー ボ 設 定  -----------------------------------------------------
//-------------------------------------------------------------------------

// 各サーボ系統の最大サーボマウント数
#define IDL_MAX 15 // L系統の最大サーボ数. 標準は15.
#define IDR_MAX 15 // R系統の最大サーボ数. 標準は15.

// L系統のサーボのマウントの設定
// 0: マウントなし        01: Single PWM,    11: I2C_PCA9685 to PWM
// 21: FUTABA_RSxTTL,   31: DYNAMIXEL 1.0, 32: DYNAMIXEL 2.0
// 43: KONDO_ICS 3.5/6, 44: KONDO_PMX,     51: JRPROPO_XBUS
// 61: FEETECH_STS,     62: FEETECH_SCS
int IDL_MT[IDL_MAX] = {
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
// 0: マウントなし        01: Single PWM,    11: I2C_PCA9685 to PWM
// 21: FUTABA_RSxTTL,   31: DYNAMIXEL 1.0, 32: DYNAMIXEL 2.0
// 43: KONDO_ICS 3.5/6, 44: KONDO_PMX,     51: JRPROPO_XBUS
// 61: FEETECH_STS,     62: FEETECH_SCS
int IDR_MT[IDR_MAX] = {
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

// L系統のサーボ回転方向補正(1:変更なし, -1:逆転)
int IDL_CW[IDL_MAX] = {
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
int IDR_CW[IDR_MAX] = {
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
float IDL_TRIM[IDL_MAX] = {
    0,      // [00]頭ヨー
    -2.36,  // [01]左肩ピッチ
    -91.13, // [02]左肩ロール
    0,      // [03]左肘ヨー
    89.98,  // [04]左肘ピッチ
    0,      // [05]左股ヨー
    0,      // [06]左股ロール
    -1.35,  // [07]左股ピッチ
    -58.05, // [08]左膝ピッチ
    -20.25, // [09]左足首ピッチ
    -0.68,  // [10]左足首ロール
    0,      // [11]追加サーボ用
    0,      // [12]追加サーボ用
    0,      // [13]追加サーボ用
    0       // [14]追加サーボ用
};

// R系統のトリム値(degree)
float IDR_TRIM[IDR_MAX] = {
    0,      // [00]腰ヨー
    0,      // [01]右肩ピッチ
    -89.44, // [02]右肩ロール
    0,      // [03]右肘ヨー
    89.98,  // [04]右肘ピッチ
    0,      // [05]右股ヨー
    1.69,   // [06]右股ロール
    -3.38,  // [07]右股ピッチ
    -57.38, // [08]右膝ピッチ
    -20.25, // [09]右足首ピッチ
    -2.36,  // [10]右足首ロール
    0,      // [11]追加サーボ用
    0,      // [12]追加サーボ用
    0,      // [13]追加サーボ用
    0,      // [14]追加サーボ用
};

//-------------------------------------------------------------------------
// 　 Meridim90 配列アクセス対応キー
//-------------------------------------------------------------------------
#define MRD_MASTER             0  // マスターコマンド
#define MRD_SEQENTIAL          1  // シーケンス番号
#define MRD_ACC_X              2  // 加速度センサX値
#define MRD_ACC_Y              3  // 加速度センサY値
#define MRD_ACC_Z              4  // 加速度センサZ値
#define MRD_GYRO_X             5  // ジャイロセンサX値
#define MRD_GYRO_Y             6  // ジャイロセンサY値
#define MRD_GYRO_Z             7  // ジャイロセンサZ値
#define MRD_MAG_X              8  // 磁気コンパスX値
#define MRD_MAG_Y              9  // 磁気コンパスY値
#define MRD_MAG_Z              10 // 磁気コンパスZ値
#define MRD_TEMP               11 // 温度センサ値
#define MRD_DIR_ROLL           12 // DMP推定ロール方向値
#define MRD_DIR_PITCH          13 // DMP推定ピッチ方向値
#define MRD_DIR_YAW            14 // DMP推定ヨー方向値
#define MRD_CONTROL_BUTTONS    15 // リモコンの基本ボタン値
#define MRD_CONTROL_STICK_L    16 // リモコンの左スティックアナログ値
#define MRD_CONTROL_STICK_R    17 // リモコンの右スティックアナログ値
#define MRD_CONTROL_L2R2ANALOG 18 // リモコンのL2R2ボタンアナログ値
#define MRD_MOTION_FRAMES      19 // モーション設定のフレーム数
#define MRD_STOP_FRAMES_MS     19 // ボード停止時のフレーム数(MCMD_STOP_BOARD_DURINGで指定)

#define C_HEAD_Y_CMD       20 // 頭ヨーのコマンド
#define C_HEAD_Y_VAL       21 // 頭ヨーの値
#define L_SHOULDER_P_CMD 22 // 左肩ピッチのコマンド
#define L_SHOULDER_P_VAL 23 // 左肩ピッチの値
#define L_SHOULDER_R_CMD 24 // 左肩ロールのコマンド
#define L_SHOULDER_R_VAL 25 // 左肩ロールの値
#define L_ELBOW_Y_CMD    26 // 左肘ヨーのコマンド
#define L_ELBOW_Y_VAL    27 // 左肘ヨーの値
#define L_ELBOW_P_CMD    28 // 左肘ピッチのコマンド
#define L_ELBOW_P_VAL    29 // 左肘ピッチの値
#define L_HIPJOINT_Y_CMD 30 // 左股ヨーのコマンド
#define L_HIPJOINT_Y_VAL 31 // 左股ヨーの値
#define L_HIPJOINT_R_CMD 32 // 左股ロールのコマンド
#define L_HIPJOINT_R_VAL 33 // 左股ロールの値
#define L_HIPJOINT_P_CMD 34 // 左股ピッチのコマンド
#define L_HIPJOINT_P_VAL 35 // 左股ピッチの値
#define L_KNEE_P_CMD     36 // 左膝ピッチのコマンド
#define L_KNEE_P_VAL     37 // 左膝ピッチの値
#define L_ANKLE_P_CMD    38 // 左足首ピッチのコマンド
#define L_ANKLE_P_VAL    39 // 左足首ピッチの値
#define L_ANKLE_R_CMD    40 // 左足首ロールのコマンド
#define L_ANKLE_R_VAL    41 // 左足首ロールの値
#define L_SERVO_ID11_CMD 42 // 追加サーボ用のコマンド
#define L_SERVO_ID11_VAL 43 // 追加サーボ用の値
#define L_SERVO_ID12_CMD 44 // 追加サーボ用のコマンド
#define L_SERVO_ID12_VAL 45 // 追加サーボ用の値
#define L_SERVO_ID13_CMD 46 // 追加サーボ用のコマンド
#define L_SERVO_ID13_VAL 47 // 追加サーボ用の値
#define L_SERVO_ID14_CMD 48 // 追加サーボ用のコマンド
#define L_SERVO_ID14_VAL 49 // 追加サーボ用の値
#define C_WAIST_Y_CMD      50 // 腰ヨーのコマンド
#define C_WAIST_Y_VAL      51 // 腰ヨーの値

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
#define R_SERVO_ID11_CMD 72 // 追加テスト用のコマンド
#define R_SERVO_ID11_VAL 73 // 追加テスト用の値
#define R_SERVO_ID12_CMD 74 // 追加テスト用のコマンド
#define R_SERVO_ID12_VAL 75 // 追加テスト用の値
#define R_SERVO_ID13_CMD 76 // 追加テスト用のコマンド
#define R_SERVO_ID13_VAL 77 // 追加テスト用の値
#define R_SERVO_ID14_CMD 78 // 追加テスト用のコマンド
#define R_SERVO_ID14_VAL 79 // 追加テスト用の値

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

#endif // __MERIDIAN_CONFIG__
