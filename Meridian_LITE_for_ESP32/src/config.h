#ifndef __MERIDIAN_CONFIG__
#define __MERIDIAN_CONFIG__

//==================================================================================================
//  MERIDIAN - LITE - ESP32の配線
//==================================================================================================
//
// ESP32devkitC  -  デバイス
//   3V3         -  BNO055 VIN
//   21          -  BNO055 SCL
//   22          -  BNO055 SDA
//   GND         -  BNO055 GND
//
// サーボIF: 3ステートバッファ版 (MRD_BOARD_SERVO_IF 0)
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
// サーボIF: SN74LXC1T45版 (MRD_BOARD_SERVO_IF 1)
//   4  EN       -  SN74LXC1T45 R系統 DIR (H:送信 A->B, L:受信 B->A)
//   13 RX       -  SN74LXC1T45 R系統 A (直結)
//   14 TX       -  SN74LXC1T45 R系統 A (直結, 受信中は出力OFF)
//   33 EN       -  SN74LXC1T45 L系統 DIR (H:送信 A->B, L:受信 B->A)
//   32 RX       -  SN74LXC1T45 L系統 A (直結)
//   27 TX       -  SN74LXC1T45 L系統 A (直結, 受信中は出力OFF)
//   3V3         -  SN74LXC1T45 VCCA
//   5V          -  SN74LXC1T45 VCCB
//   GND         -  SN74LXC1T45 GND
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
//   13          -  OK/LED? (SN74LXC1T45版ではサーボR系統 RX)
//   14          -  OK/(SPI HS CLK) (SN74LXC1T45版ではサーボR系統 TX)
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

// Meridian Boardのバージョン(サーボIF回路の違い)
// 1: Ver L1.43以上 (SN74LXC1T45版), 0: それ以前もしくは無印 (3ステートバッファ版)
#define MRD_BOARD_SERVO_IF 1

// Meridimの基本設定
#define MRDM_LEN        90  // Meridim配列の長さ設定(デフォルトは90)
#define FRAME_DURATION  10  // 1フレームあたりの単位時間(単位ms, デフォルトは10)
#define CHARGE_TIME     200 // 起動時のコンデンサチャージ待機時間(単位ms)
#define MRD_L_ORIGIDX   20  // Meridim配列のL系統の最初のインデックス(デフォルトは20)
#define MRD_R_ORIGIDX   50  // Meridim配列のR系統の最初のインデックス(デフォルトは50)
#define MRD_SERVO_SLOTS 15  // Meridim配列の1系統あたりの最大接続サーボ数(デフォルトは15)

// 各種ハードウェアのマウント有無
#define MOUNT_SD      1           // SDカードリーダーの有無(0:なし, 1:あり)
#define MOUNT_IMUAHRS BNO055_AHRS // IMU/AHRSの搭載 NO_IMU, MPU6050_IMU, MPU9250_IMU(未対応), BNO055_AHRS
#define MOUNT_PAD     PC          // ジョイパッドの搭載 PC, KRR5FH, WIIMOTE, WIIMOTE_C, SBDBT, BLUERETRO (MERIMOTEは未対応)

// 動作モード
#define MODE_ESP32_STANDALONE 0 // ESP32をボードに挿さず動作確認(0:NO, 1:YES)
#define MODE_UDP_RECEIVE      1 // PCからのデータ受信(0:OFF, 1:ON, 通常は1)
#define MODE_UDP_SEND         1 // PCへのデータ送信(0:OFF, 1:ON, 通常は1)
#define MODE_UDP_PASSIVE      0 // 通信の周期(0:ボード主導. FRAME_DURATIONごとに送り, 受信を待つ,
                                //   1:PC主導. 受信したらすぐ処理して返信し, フレームの時刻を待たない.
                                //   FRAME_DURATION 待っても受信がなければ, 前回の受信データで処理して返信する).
                                //   MODE_UDP_RECEIVE 0 のときはボード主導. 動作中は 10005 / 10006 で切り替えられる

// Wifi/有線LANの設定(SSID, パスワード, 固定IP, MACアドレス等は別途keys.hで指定)
#define MODE_ETHER      0    // WiFiか有線LANか(0:wifi, 1:有線LAN, 通常は0)
#define MODE_FIXED_IP   0    // WiFi用IPアドレスを固定するか(0:NO, 1:YES)
#define UDP_TIMEOUT_US  4000 // UDPの待受タイムアウト(単位us)
#define UDP_POLL_US     100  // UDP待受中の受信確認間隔(単位us)
#define UDP_FAILSAFE_MS 1000 // PCからの受信が途絶えてから全サーボを脱力するまでの時間(単位ms. 0なら脱力しない)
#define UDP_SELF_SPEED  60.0 // PCから受信しない場合(MODE_UDP_RECEIVE 0)の, サーボを目標へ動かす速さの上限(単位degree/s)

// EEPROMの設定. EEPROMは1ページ180byte(Meridim1つ分)で, ページごとに config と EEPROM のどちらの値を使うか選べる
#define EEPROM_SIZE       2880 // 使用するEEPROMのサイズ(バイト. 180byte × 16ページ)
#define EEPROM_SET        1    // 起動時に, configの値を使うページをconfigの値でEEPROMに書く(0:書かない, 1:書く)
#define EEPROM_PROTECT    0    // EEPROMの書き込み保護(0:保護しない, 1:書き込み禁止)
#define EEPROM_LOAD       0    // EEPROMの値を使うページ(下の EEPROM_P_* の和. 0:すべてconfig, 255:すべてEEPROM)
#define EEPROM_DUMP       0    // 起動時のEEPROMデータのダンプ表示
#define EEPROM_STYLE      Dec  // 起動時のEEPROM内容のダンプ表示の書式(Bin,Hex,Dec)
#define EEPROM_P_NET      1    // P0 ネットワーク(keys.h の値. WiFi の SSID・パスワードなど)
#define EEPROM_P_SERVO    2    // P1 サーボのスロット(マウント, ID, 回転方向, トリム)
#define EEPROM_P_SLOT_EXT 4    // P2 スロットの拡張(種類, 8bitのID)
#define EEPROM_P_LINE     8    // P3 サーボの系統と共通設定
#define EEPROM_P_SYSTEM   16   // P4 システム・動作モード
#define EEPROM_P_PWM      32   // P5 PWMの共通設定と対応表
#define EEPROM_P_PWM_L    64   // P6 PWMのL系統のスロット(中央, +90deg, 上限, 下限)
#define EEPROM_P_PWM_R    128  // P7 PWMのR系統のスロット

// 動作チェックモード
#define CHECK_SD_RW     1 // 起動時のSDカードリーダーの読み書きチェック
#define CHECK_EEPROM_RW 0 // 起動時のEEPROMの動作チェック ※LITEでは未使用

// シリアルモニタリング
#define MONITOR_FRAME_DELAY       0    // シリアルモニタでフレーム処理時間と遅延を1秒ごとに表示(0:OFF, 1:ON)
#define MONITOR_FLOW              0    // シリアルモニタでフローを表示(0:OFF, 1:ON)
#define MONITOR_ERR_SERVO         0    // シリアルモニタでサーボエラーを表示(0:OFF, 1:ON) ※LITEでは未使用
#define MONITOR_ERR_ALL           0    // シリアルモニタで全経路の受信エラー回数を毎フレーム表示(0:OFF, 1:ON)
#define MONITOR_SEQ               0    // シリアルモニタでシーケンス番号チェックを表示(0:OFF, 1:ON)
#define MONITOR_PAD               0    // シリアルモニタでリモコンのデータを表示(0:OFF, 1:ON) ※LITEでは未使用
#define MONITOR_UDP_TIMEOUT       0    // シリアルモニタでUDP受信のタイムアウトを表示(0:OFF, 1:ON)
#define MONITOR_SUPPRESS_DURATION 8000 // 起動直後のタイムアウトメッセージ抑制時間(単位ms)

// I2C設定, I2Cセンサ関連設定
#define I2C0_SPEED       400000 // I2Cの速度(400kHz推奨)
#define IMUAHRS_INTERVAL 10     // IMU/AHRSのセンサの読み取り間隔(ms)
#define IMUAHRS_STOCK    4      // MPUで移動平均を取る際の元にする時系列データの個数 ※LITEでは未使用

// INA226(電流・電圧センサ. I2C0. IMUと同じバス). 電圧を Meridim[86], 電流を Meridim[87] に入れる
#define MOUNT_INA226      0     // INA226の搭載(0:なし, 1:あり)
#define INA226_ADDR       0x40  // I2Cアドレス(A0/A1 で 0x40-0x4F)
#define INA226_SHUNT_MOHM 100.0 // シャント抵抗(mΩ). 測れる電流は ±81.92mV / シャント抵抗 (100mΩ で ±0.82A, 2mΩ で ±41A)
#define INA226_INTERVAL   10    // 読み取り間隔(ms). INA226 は約9ms ごとに値を更新する(平均4回, 変換1.1ms)
// #define I2C1_MERIMOTE_ADDR 0x58 // MerimoteのI2Cアドレス

// SPI設定
#define SPI0_SPEED 6000000 // SPI通信の速度(6000000Hz推奨) ※LITEでは起動時の表示のみに使う

// PC接続関連設定
#define SERIAL_PC_BPS     921600 // PCとのシリアル速度(モニタリング表示用. platformio.iniのmonitor_speedと合わせる)
#define SERIAL_PC_TIMEOUT 2000   // PCとのシリアル接続確立タイムアウト(ms) ※LITEでは未使用(ESP32ではシリアルの接続を検出できない)

// JOYPAD関連設定
#define PAD_INIT_TIMEOUT  10000 // 起動時のJOYPADの接続確立のタイムアウト(ms)
#define PAD_INTERVAL      10    // JOYPADのデータを読みに行く間隔(ms) (※KRC-5FHでは4推奨)
#define PAD_BUTTON_MARGE  1     // 0:JOYPADのボタンデータでMeridim受信値を上書き, 1:Meridim受信値に論理和
#define PAD_GENERALIZE    1     // ジョイパッドの入力値をPS系に一般化する
#define PAD_SBDBT_BPS     2400  // SBDBT の通信速度(bps. RCB3形式. SBDBT のファームウェアの BAUDRATE1 と合わせる)
#define PAD_BLUERETRO_I2C 1     // BlueRetro をつなぐI2C(0:I2C0. IMUと同じバス, 1:I2C1. PIN_I2C1_SDA/SCL)
#define PAD_SBDBT_TIMEOUT 0     // SBDBT の電文がこの時間届かなければパッドの値を0にする(ms. 0:しない. SBDBT が毎回送る設定のときに使う)

// ピンアサイン
#define PWM_PIN_MODE 0  // ピン25/26の用途(0:LED(従来), 1:本体PWMサーボ2ch, 2:PCA9685用I2C(Wire1, SDA=25, SCL=26))
                        // ※1/2の場合, エラーLED(25)とBT LED(26)は使えない
#define PIN_ERR_LED  25 // エラーLED(起動時の設定エラー等で停止したときに点滅). -1:なし. 本体PWM・I2C1・SBDBTの受信と同じピンなら使わない
#define PIN_EN_L     33 // サーボL系統のENピン
#define PIN_TX_L     27 // サーボL系統のTXピン(Serial1)
#define PIN_RX_L     32 // サーボL系統のRXピン(Serial1)
#define PIN_EN_R     4  // サーボR系統のENピン
#if MRD_BOARD_SERVO_IF == 1
#define PIN_TX_R 14 // サーボR系統のTXピン(Serial2)
#define PIN_RX_R 13 // サーボR系統のRXピン(Serial2)
#else
#define PIN_TX_R 17 // サーボR系統のTXピン(Serial2)
#define PIN_RX_R 16 // サーボR系統のRXピン(Serial2)
#endif
#define PIN_CHIPSELECT_SD  15 // SDカード用のCSピン
#define PIN_CHIPSELECT_LAN 5  // 有線LAN用のCSピン
#define PIN_RESET_LAN      14 // W5500リセットピン(※ボード裏から半田付けにてフリーピンに配線) ※SN74LXC1T45版ではPIN_TX_Rと重複, 未使用
#define PIN_I2C0_SDA       22 // I2CのSDAピン
#define PIN_I2C0_SCL       21 // I2CのSCLピン
#define PIN_LED_BT         26 // Bluetooth接続確認用ピン(点滅はペアリング,点灯でリンク確立). -1:なし. 同上
#define PIN_PWM_0          25 // 本体PWMサーボ0 (PWM_PIN_MODE 1)
#define PIN_PWM_1          26 // 本体PWMサーボ1 (PWM_PIN_MODE 1)
#define PIN_I2C1_SDA       25 // I2C1(Wire1)のSDA (PCA9685 の PWM_PIN_MODE 2, BlueRetro の PAD_BLUERETRO_I2C 1)
#define PIN_I2C1_SCL       26 // I2C1(Wire1)のSCL (PCA9685 の PWM_PIN_MODE 2, BlueRetro の PAD_BLUERETRO_I2C 1)
#define PIN_PAD_RX         34 // SBDBT の TX をつなぐピン(MOUNT_PAD SBDBT. ソフトウェアで受信. 入力専用ピン可)

//-------------------------------------------------------------------------
// サーボ設定
//-------------------------------------------------------------------------

// コマンドサーボの種類
// 00: NOSERVO (マウントなし),            01: PWM_S (本体ピンPWM)
// 11: PCA9685 (I2C_PCA9685toPWM),      21: FTBRSX (FUTABA_RSxTTL, RS30x)
// 31: DXL1 (DYNAMIXEL 1.0, AX/RX/MX), 32: DXL2 (DYNAMIXEL 2.0, Xシリーズ)
// 43: KOICS3 (KONDO_ICS 3.5 / 3.6),    44: KOPMX (KONDO_PMX, RS-485変換経由)
// 51: JRXBUS (JRPROPO_XBUS),          71: HTCHSR (Hitec_HSR)
// 61: FTCSTS (FEETECH_STS),            62: FTCSCS (FEETECH_SCS) ※STSとSCSは別物. 混在時は位置を交互に読む
#define MOUNT_SERVO_TYPE_L 43 // L系統のコマンドサーボの種類
#define MOUNT_SERVO_TYPE_R 43 // R系統のコマンドサーボの種類
// 1系統に2種類目のサーボを混在させる場合の種類(0:混在なし). スロットごとの種類は IXL_MT/IXR_MT で指定する.
// 混在できるのは 21(FUTABA), 31(DYNAMIXEL 1.0), 32(DYNAMIXEL 2.0), 61/62(FEETECH) の異なる2種類のみ. 現在位置は種類ごとに交互に読む
#define MOUNT_SERVO_TYPE_L2 0 // L系統の2種類目のサーボの種類
#define MOUNT_SERVO_TYPE_R2 0 // R系統の2種類目のサーボの種類

// サーボ関連設定
#define SERVO_BAUDRATE_L        1250000                   // L系統のサーボの通信速度bps(ICS:1250000, FEETECH:1000000, DXL:サーボの設定値(AXは1M, MX/RXは57600), FUTABA:460800推奨, XBUS:250000, HITEC:19200, PMX:3000000推奨)
#define SERVO_BAUDRATE_R        1250000                   // R系統のサーボの通信速度bps(ICS:1250000, FEETECH:1000000, DXL:サーボの設定値(AXは1M, MX/RXは57600), FUTABA:460800推奨, XBUS:250000, HITEC:19200, PMX:3000000推奨)
#define SERVO_BAUDRATE_L2       1000000                   // L系統の2種類目のサーボの通信速度bps(MOUNT_SERVO_TYPE_L2 が0なら使わない)
#define SERVO_BAUDRATE_R2       1000000                   // R系統の2種類目のサーボの通信速度bps(MOUNT_SERVO_TYPE_R2 が0なら使わない)
#define SERVO_TIMEOUT_L         2                         // L系統のサーボ返信待ちのタイムアウト時間(ms)
#define SERVO_TIMEOUT_R         2                         // R系統のサーボ返信待ちのタイムアウト時間(ms)
#define SERVO_ECHO_L            (MRD_BOARD_SERVO_IF == 1) // L系統の送信エコー処理(0:なし/3ステート版, 1:あり/SN74LXC1T45版, 2:起動時に自動判定). 既定は基板の設定に従う
#define SERVO_ECHO_R            (MRD_BOARD_SERVO_IF == 1) // R系統の送信エコー処理(同上)
#define SERVO_TX_RELEASE        (MRD_BOARD_SERVO_IF == 1) // 受信中にTXピンの出力を切る(TX/RXが1T45のA端子に直結の基板用)
#define SERVO_LOST_ERR_WAIT     6                         // 連続何フレームサーボ信号をロストしたら異常とするか
#define SERVO_STEP_MAX          15.0                      // 1フレームあたりの指令値の最大変化量(degree)
#define SERVO_UNMOUNT_REPLY     0                         // マウントしていないサーボのスロットへの返信値(0:0を返す, 1:受信した値をそのまま返す)
#define SERVO_MOTION_FRAMES_MAX 1000                      // Meridim[19](移動に使うフレーム数)の上限. 0なら補間しない(受信した目標へ即時)

// FEETECH(STS/SCS)サーボ関連設定. 系列(STS/SCS)はスロットの種類(61/62)で決まる. 起動時にサーボの値と違えば警告を表示する
#define SERVO_FTC_STS_RANGE        360.0 // STSの位置値0-4095に対応する角度範囲(degree)
#define SERVO_FTC_SCS_RANGE        300.0 // SCSの位置値0-1023に対応する角度範囲(degree, SCS0009等. 機種により異なる)
#define SERVO_FTC_SCS_SPEED        1500  // SCSの目標速度(位置値/秒). SCSは0だと駆動しない. 1500で約440deg/s(300deg機種)
#define SERVO_FTC_FIX_RETURN_DELAY 1     // 起動時に応答遅延時間が0でなければ0に書き換える(0:しない, 1:する)
                                         // ※サーボのEEPROMに保存される. 0でないと1系統15台で100Hzを維持できない

// DYNAMIXEL Protocol 2.0 (Xシリーズ)サーボ関連設定. 通信速度は事前にDYNAMIXEL Wizard等で1Mbps以上にしておく
#define SERVO_DXL_FIX_RETURN_DELAY 1 // 起動時に応答遅延時間(初期値500us)が0でなければ0に書き換える(0:しない, 1:する)
                                     // ※サーボのEEPROMに保存される. 書き換えの際は一度トルクOFFにする

// JR XBUSサーボ関連設定. 通信速度は250000bps固定. 50Hzで全サーボへ送信(戻り値なし, 指令値を返す)
#define SERVO_XBUS_ANGLE180 0 // サーボ側の180度モードに合わせる(0:通常 ±60deg=900-2100us, 1:180度モード ±90deg)

// Hitec HSRサーボ関連設定. 通信速度は19200bps固定. 1台ずつ処理する
//   動作中(トルクONのサーボあり): 位置を送るだけ(返信は待たず, 指令値を返す)
//   脱力中(全サーボOFF): 現在位置を読むだけ(角度値は送らず, 読み取った角度を返す)
#define SERVO_HTC_PER_FRAME  1    // 1フレームに位置を送る/読むサーボ数(1台約4.2ms). 送る場合は各サーボ50Hzまで
                                  // 1なら2台まで各50Hz, 4台で各25Hz
#define SERVO_HTC_US_PER_DEG 10.0 // 1degあたりのパルス幅(us). 1500us中心, 550-2450us
#define SERVO_HTC_SPEED      0xFF // 現在位置の読み出し(0xE9)で同時に設定される速度(1-255. 0xFFはEEPROMの初期値)

// PWMサーボ(本体ピン25/26, PCA9685)関連設定. 50Hz, 動作命令のみ(戻り値なし). ピンの用途は PWM_PIN_MODE で選ぶ
// パルス幅と角度の範囲はスロットごと(IXL_PWM_CENTER などの配列)
#define I2C1_SPEED     400000   // I2C1(Wire1. PCA9685, BlueRetro)の速度
#define PCA9685_NUM    1        // PCA9685の枚数(0-2). 1枚目は偶数フレーム, 2枚目は奇数フレームで更新(各50Hz)
#define PCA9685_ADDR_0 0x40     // 1枚目のI2Cアドレス
#define PCA9685_ADDR_1 0x41     // 2枚目のI2Cアドレス(A0をHIGHに)
#define PCA9685_OSC_HZ 25000000 // 内蔵発振器の周波数. 個体差でパルス幅がずれる場合に実測値に合わせる

// FUTABA RS30xサーボ関連設定. 通信速度は最大460800bps(初期値115200bps)
#define SERVO_FTB_READ_PER_FRAME 8 // 1フレームに現在位置を読むサーボ数(超える分は次のフレームで順に読む)
                                   // 460800bpsで1台あたり約0.5ms. 15台を毎フレーム読むと100Hzに収まらない

// 各サーボ系統の最大サーボマウント数
#define IXL_MAX 15 // L系統の最大サーボ数. 標準は15.
#define IXR_MAX 15 // R系統の最大サーボ数. 標準は15.

// L系統のサーボのマウントの設定(番号は「コマンドサーボの種類」を参照. 0:マウントなし)
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

// R系統のサーボのマウントの設定(番号は「コマンドサーボの種類」を参照. 0:マウントなし)
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
float IXL_TRIM[IXL_MAX] = {
    0.0, // [00]頭ヨー
    0.0, // [01]左肩ピッチ
    0.0, // [02]左肩ロール
    0.0, // [03]左肘ヨー
    0.0, // [04]左肘ピッチ
    0.0, // [05]左股ヨー
    0.0, // [06]左股ロール
    0.0, // [07]左股ピッチ
    0.0, // [08]左膝ピッチ
    0.0, // [09]左足首ピッチ
    0.0, // [10]左足首ロール
    0.0, // [11]追加サーボ用
    0.0, // [12]追加サーボ用
    0.0, // [13]追加サーボ用
    0.0, // [14]追加サーボ用
};

// R系統のトリム値(degree)
float IXR_TRIM[IXR_MAX] = {
    -4.28, // [00]腰ヨー
    0.0,   // [01]右肩ピッチ
    0.0,   // [02]右肩ロール
    0.0,   // [03]右肘ヨー
    0.0,   // [04]右肘ピッチ
    0.0,   // [05]右股ヨー
    0.0,   // [06]右股ロール
    0.0,   // [07]右股ピッチ
    0.0,   // [08]右膝ピッチ
    0.0,   // [09]右足首ピッチ
    0.0,   // [10]右足首ロール
    0.0,   // [11]追加サーボ用
    0.0,   // [12]追加サーボ用
    0.0,   // [13]追加サーボ用
    0.0,   // [14]追加サーボ用
};

// PWMサーボのスロットごとの設定(種類1/11のスロットと, PWM_PIN_SLOT/PCA9685_SLOT で指定したスロットで使う).
// パルス幅 = 中央 + サーボ側の角度 × (+90°のパルス − 中央) / 90. −90°は中央に対して対称(2×中央 − +90°のパルス).
// サーボ側の角度 = トリム + 指令の角度 × 回転方向. 上限/下限を超える指令は上限/下限で止める.

// L系統: 0degのパルス幅(us)
int IXL_PWM_CENTER[IXL_MAX] = {
    1500, // [00]頭ヨー
    1500, // [01]左肩ピッチ
    1500, // [02]左肩ロール
    1500, // [03]左肘ヨー
    1500, // [04]左肘ピッチ
    1500, // [05]左股ヨー
    1500, // [06]左股ロール
    1500, // [07]左股ピッチ
    1500, // [08]左膝ピッチ
    1500, // [09]左足首ピッチ
    1500, // [10]左足首ロール
    1500, // [11]追加サーボ用
    1500, // [12]追加サーボ用
    1500, // [13]追加サーボ用
    1500, // [14]追加サーボ用
};

// L系統: +90degのパルス幅(us). 中央より小さくすると逆回転
int IXL_PWM_P90[IXL_MAX] = {
    2500, // [00]頭ヨー
    2500, // [01]左肩ピッチ
    2500, // [02]左肩ロール
    2500, // [03]左肘ヨー
    2500, // [04]左肘ピッチ
    2500, // [05]左股ヨー
    2500, // [06]左股ロール
    2500, // [07]左股ピッチ
    2500, // [08]左膝ピッチ
    2500, // [09]左足首ピッチ
    2500, // [10]左足首ロール
    2500, // [11]追加サーボ用
    2500, // [12]追加サーボ用
    2500, // [13]追加サーボ用
    2500, // [14]追加サーボ用
};

// L系統: サーボ側の角度の上限(degree)
float IXL_PWM_MAX_DEG[IXL_MAX] = {
    90.0, // [00]頭ヨー
    90.0, // [01]左肩ピッチ
    90.0, // [02]左肩ロール
    90.0, // [03]左肘ヨー
    90.0, // [04]左肘ピッチ
    90.0, // [05]左股ヨー
    90.0, // [06]左股ロール
    90.0, // [07]左股ピッチ
    90.0, // [08]左膝ピッチ
    90.0, // [09]左足首ピッチ
    90.0, // [10]左足首ロール
    90.0, // [11]追加サーボ用
    90.0, // [12]追加サーボ用
    90.0, // [13]追加サーボ用
    90.0, // [14]追加サーボ用
};

// L系統: サーボ側の角度の下限(degree)
float IXL_PWM_MIN_DEG[IXL_MAX] = {
    -90.0, // [00]頭ヨー
    -90.0, // [01]左肩ピッチ
    -90.0, // [02]左肩ロール
    -90.0, // [03]左肘ヨー
    -90.0, // [04]左肘ピッチ
    -90.0, // [05]左股ヨー
    -90.0, // [06]左股ロール
    -90.0, // [07]左股ピッチ
    -90.0, // [08]左膝ピッチ
    -90.0, // [09]左足首ピッチ
    -90.0, // [10]左足首ロール
    -90.0, // [11]追加サーボ用
    -90.0, // [12]追加サーボ用
    -90.0, // [13]追加サーボ用
    -90.0, // [14]追加サーボ用
};

// R系統: 0degのパルス幅(us)
int IXR_PWM_CENTER[IXR_MAX] = {
    1500, // [00]腰ヨー
    1500, // [01]右肩ピッチ
    1500, // [02]右肩ロール
    1500, // [03]右肘ヨー
    1500, // [04]右肘ピッチ
    1500, // [05]右股ヨー
    1500, // [06]右股ロール
    1500, // [07]右股ピッチ
    1500, // [08]右膝ピッチ
    1500, // [09]右足首ピッチ
    1500, // [10]右足首ロール
    1500, // [11]追加サーボ用
    1500, // [12]追加サーボ用
    1500, // [13]追加サーボ用
    1500, // [14]追加サーボ用
};

// R系統: +90degのパルス幅(us). 中央より小さくすると逆回転
int IXR_PWM_P90[IXR_MAX] = {
    2500, // [00]腰ヨー
    2500, // [01]右肩ピッチ
    2500, // [02]右肩ロール
    2500, // [03]右肘ヨー
    2500, // [04]右肘ピッチ
    2500, // [05]右股ヨー
    2500, // [06]右股ロール
    2500, // [07]右股ピッチ
    2500, // [08]右膝ピッチ
    2500, // [09]右足首ピッチ
    2500, // [10]右足首ロール
    2500, // [11]追加サーボ用
    2500, // [12]追加サーボ用
    2500, // [13]追加サーボ用
    2500, // [14]追加サーボ用
};

// R系統: サーボ側の角度の上限(degree)
float IXR_PWM_MAX_DEG[IXR_MAX] = {
    90.0, // [00]腰ヨー
    90.0, // [01]右肩ピッチ
    90.0, // [02]右肩ロール
    90.0, // [03]右肘ヨー
    90.0, // [04]右肘ピッチ
    90.0, // [05]右股ヨー
    90.0, // [06]右股ロール
    90.0, // [07]右股ピッチ
    90.0, // [08]右膝ピッチ
    90.0, // [09]右足首ピッチ
    90.0, // [10]右足首ロール
    90.0, // [11]追加サーボ用
    90.0, // [12]追加サーボ用
    90.0, // [13]追加サーボ用
    90.0, // [14]追加サーボ用
};

// R系統: サーボ側の角度の下限(degree)
float IXR_PWM_MIN_DEG[IXR_MAX] = {
    -90.0, // [00]腰ヨー
    -90.0, // [01]右肩ピッチ
    -90.0, // [02]右肩ロール
    -90.0, // [03]右肘ヨー
    -90.0, // [04]右肘ピッチ
    -90.0, // [05]右股ヨー
    -90.0, // [06]右股ロール
    -90.0, // [07]右股ピッチ
    -90.0, // [08]右膝ピッチ
    -90.0, // [09]右足首ピッチ
    -90.0, // [10]右足首ロール
    -90.0, // [11]追加サーボ用
    -90.0, // [12]追加サーボ用
    -90.0, // [13]追加サーボ用
    -90.0, // [14]追加サーボ用
};

// PWM出力とMeridimスロットの対応表. SV_L(n)/SV_R(n)でL/R系統のn番スロットの指令に従う. SV_NONEは未使用.
// スロットのコマンドが1なら角度を出力, 0/2ならパルスを止める(通常以外のスロットは直前の出力を保持). トリムと回転方向はスロットの設定を使う.
// UARTのサーボと共有しないスロットは, IXL_MT/IXR_MTの種類を1(本体PWM)/11(PCA9685)にする(指令値をそのまま返す).
#define SV_L(n) (n)
#define SV_R(n) (100 + (n))
#define SV_NONE (-1)
int PWM_PIN_SLOT[2] = {SV_NONE, SV_NONE}; // 本体PWMサーボ [0]:ピン25, [1]:ピン26 (PWM_PIN_MODE 1)
int PCA9685_SLOT[2][16] = {               // PCA9685 [枚][ch] (PWM_PIN_MODE 2)
                           {SV_NONE, SV_NONE, SV_NONE, SV_NONE, SV_NONE, SV_NONE, SV_NONE, SV_NONE, SV_NONE, SV_NONE, SV_NONE, SV_NONE, SV_NONE, SV_NONE, SV_NONE, SV_NONE},
                           {SV_NONE, SV_NONE, SV_NONE, SV_NONE, SV_NONE, SV_NONE, SV_NONE, SV_NONE, SV_NONE, SV_NONE, SV_NONE, SV_NONE, SV_NONE, SV_NONE, SV_NONE, SV_NONE}};

// サーボのリンク. フォロワーのスロットに, リーダーのスロットの指令を毎フレームコピーする.
// {フォロワー, リーダー, 向き}. スロットは SV_L(n)/SV_R(n) で指定する(L/Rをまたいでもよい). 行はいくつでも足せる.
// 向き 1: 同じ値, -1: 符号を反転. 角度(degree)と車輪の速度に掛け, トルクON/OFFと通常/車輪の別はそのままコピーする.
// コピーするのはMeridimの値(トリム・回転方向を掛ける前)で, フォロワーのトリムと IXL_CW/IXR_CW はそのまま効く.
// PCがフォロワーのスロットに送った値は使わない. ただしパラメータ系コマンドとスルーは, フォロワー宛てでも実行する.
// リーダーがパラメータ系コマンド・スルーの間は, フォロワーは直前の指令を保つ. 返信はフォロワー自身の現在位置.
// 使わない行は {SV_NONE, SV_NONE, 1}. 範囲外のスロット, 同じスロットどうし, 他の行のリーダーかフォロワーを
// フォロワーにする行は, 起動時に警告して使わない. EEPROM には保存しない.
int SERVO_LINK[][3] = {
    {SV_NONE, SV_NONE, 1}, // 例: {SV_L(4), SV_L(3), -1} で L04 を L03 と逆向きに動かす
};

//-------------------------------------------------------------------------
//  固定値, マスターコマンド定義
//-------------------------------------------------------------------------
// Meridian共通の定義. 「※LITEでは未使用」の定数は他ボードとの互換のために残している
#define MCMD_TORQUE_ALL_OFF         0      // すべてのサーボトルクをオフにする(脱力)
#define MCMD_DUMMY_DATA             -32768 // SPI送受信用のダミーデータ判定用 ※LITEでは未使用
#define MCMD_TEST_VALUE             -32767 // テスト用の仮設変数 ※LITEでは未使用
#define MCMD_SENSOR_YAW_CALIB       10002  // センサの推定ヨー軸を現在値センターとしてリセット
#define MCMD_SENSOR_ALL_CALIB       10003  // センサの3軸について現在値を原点としてリセット
#define MCMD_ERR_CLEAR_SERVO_ID     10004  // 通信エラーのサーボのIDをクリア(MRD_ERR_l)
#define MCMD_BOARD_TRANSMIT_ACTIVE  10005  // ボードが定刻で送信を行うモード(PC側が受信待ち). ボード主導にする
#define MCMD_BOARD_TRANSMIT_PASSIVE 10006  // ボードが受信を待ち返信するモード(PC側が定刻送信). PC主導にする(MODE_UDP_PASSIVE)
#define MCMD_FRAMETIMER_RESET       10007  // フレームタイマーを現在時刻にリセット
#define MCMD_BOARD_STOP_DURING      10008  // ボードの末端処理を[MRD_STOP_FRAMES]ミリ秒止める(最大1000ms)
#define MCMD_EEPROM_ENTER_WRITE     10009  // EEPROM書き込みモードのスタート ※LITEではフレームタイマーのリセットのみ(書き込みは10300+p)
#define MCMD_EEPROM_EXIT_WRITE      10010  // EEPROM書き込みモードの終了 ※LITEでは未使用
#define MCMD_EEPROM_ENTER_READ      10011  // EEPROM読み出しモードのスタート ※LITEでは未使用
#define MCMD_EEPROM_EXIT_READ       10012  // EEPROM読み出しモードの終了 ※LITEでは未使用
#define MCMD_SDCARD_ENTER_WRITE     10013  // SDCARD書き込みモードのスタート ※LITEでは未使用
#define MCMD_SDCARD_EXIT_WRITE      10014  // SDCARD書き込みモードの終了 ※LITEでは未使用
#define MCMD_SDCARD_ENTER_READ      10015  // SDCARD読み出しモードのスタート ※LITEでは未使用
#define MCMD_SDCARD_EXIT_READ       10016  // SDCARD読み出しモードの終了 ※LITEでは未使用
#define MCMD_START_TRIM_SETTING     10100  // トリム設定モードに入る(Meridian_console.py連携)
#define MCMD_EEPROM_SAVE_TRIM       10101  // 受信したスロットの設定ワードとトリムをEEPROMのP1に書き込む
#define MCMD_EEPROM_LOAD_TRIM       10102  // EEPROMのトリム値をサーボに反映する
#define MCMD_EEPROM_BOARDTOPC_DATA0 10200  // EEPROMの[0][*]をボードからPCにMeridimで送信する(+p でページp. LITEは0-15)
#define MCMD_EEPROM_BOARDTOPC_DATA1 10201  // EEPROMの[1][*]をボードからPCにMeridimで送信する
#define MCMD_EEPROM_BOARDTOPC_DATA2 10202  // EEPROMの[2][*]をボードからPCにMeridimで送信する
#define MCMD_EEPROM_PCTOBOARD_DATA0 10300  // EEPROM用の[0][*]をPCからボードにMeridimで送信する(+p でページp. LITEは0-15)
#define MCMD_EEPROM_PCTOBOARD_DATA1 10301  // EEPROM用の[1][*]をPCからボードにMeridimで送信する
#define MCMD_EEPROM_PCTOBOARD_DATA2 10302  // EEPROM用の[2][*]をPCからボードにMeridimで送信する
#define MCMD_BOARD_RESTART          10400  // ボードを再起動する(全スロットのコマンドが0のときだけ) ※LITEの拡張

#define MCMD_NAK 32766 // コマンド実行の失敗を応答(EEPROMの書き込みの失敗)
#define MCMD_ACK 32767 // コマンド実行の成功を応答 ※LITEでは未使用

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
#define MRD_STOP_FRAMES   19 // ボード停止時の時間ms(MCMD_BOARD_STOP_DURINGで指定)
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
#define MRD_USERDATA_86 86 // ユーザー定義用(MOUNT_INA226 1 ではボードが INA226 の電圧を書く)
#define MRD_USERDATA_87 87 // ユーザー定義用(MOUNT_INA226 1 ではボードが INA226 の電流を書く)
#define MRD_INA226_VOLT 86 // INA226 のバス電圧(V × 100)
#define MRD_INA226_CURR 87 // INA226 の電流(A × 100. シャントの向きで符号)
// #define MRD_ERR         88 // エラーコード (MRDM_LEN - 2)
// #define MRD_CKSM        89 // チェックサム (MRDM_LEN - 1)

// エラービット MRD_ERR の上位8bit分
#define ERRBIT_15_ESP_PC       15 // ESP32 → PC のUDP受信エラー (0:エラーなし、1:エラー検出) ※LITEでは未使用
#define ERRBIT_14_PC_ESP       14 // PC → ESP32 のUDP受信エラー
#define ERRBIT_13_ESP_TSY      13 // ESP32 → TeensyのSPI受信エラー ※LITEでは未使用
#define ERRBIT_12_TSY_ESP      12 // Teensy → ESP32 のSPI受信エラー ※LITEでは未使用
#define ERRBIT_11_BOARD_DELAY  11 // Teensy or ESP32の処理ディレイ (末端で捕捉) ※LITEでは未使用
#define ERRBIT_10_UDP_ESP_SKIP 10 // PC → ESP32 のUDPフレームスキップエラー
#define ERRBIT_9_BOARD_SKIP    9  // PC → ESP32 → Teensy のフレームスキップエラー(末端で捕捉) ※LITEでは未使用
#define ERRBIT_8_PC_SKIP       8  // Teensy → ESP32 → PC のフレームスキップエラー(末端で捕捉) ※LITEでは未使用

#endif // __MERIDIAN_CONFIG__
