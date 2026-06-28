#ifndef __MERIDIAN_EEPROM_H__
#define __MERIDIAN_EEPROM_H__

// ヘッダファイルの読み込み
#include "../config.h"
#include "mrd_types.h"
#include "mrd_util.h"
#include <Arduino.h>

// ライブラリ導入
#include <EEPROM.h>

//------------------------------------------------------------------------------------
//  定数
//------------------------------------------------------------------------------------

// EEPROMバージョン: major.minor.年月 → 12606 = v1.2.June2026
#define EEPROM_VERSION 12606

// EEPROMマップ ワードインデックス
#define EEP_W_HEADER    0   // LEN(upper) / 0x55識別子(lower)
#define EEP_W_VERSION   1   // EEPROMバージョン
#define EEP_W_WRCNT_H   2   // 書き込みカウンタ上位word
#define EEP_W_WRCNT_L   3   // 書き込みカウンタ下位word
#define EEP_W_BOOT_MOT  4   // 起動時モーション番号
#define EEP_W_NET_MODE  5   // ネットワーク設定(upper=profile, lower=mode)
#define EEP_W_SSID      6   // SSID先頭(word 6-21, 16words=32chars)
#define EEP_W_PASS      22  // PASS先頭(word 22-53, 32words=64chars)
#define EEP_W_SEND_IP   54  // 送信先PC IP(word 54-55)
#define EEP_W_FIXED_IP  56  // ESP32固定IP(word 56-57)
#define EEP_W_GATEWAY   58  // ゲートウェイIP(word 58-59)
#define EEP_W_SUBNET    60  // サブネットマスク(word 60-61)
#define EEP_W_SEND_PORT 62  // UDP送信ポート
#define EEP_W_RECV_PORT 63  // UDP受信ポート
#define EEP_W_UDP_TMOUT 64  // UDP待受タイムアウト(lower byte)
#define EEP_W_I2C0_SPD  65  // I2C_0速度(/1000)
#define EEP_W_I2C1_SPD  66  // I2C_1速度(/1000)
#define EEP_W_SPI0_SPD  67  // SPI_0速度(/1000)
#define EEP_W_SPI1_SPD  68  // SPI_1速度(/1000)
#define EEP_W_CHARGE    69  // 起動待ち時間(ms)
#define EEP_W_SD_FLAGS  70  // SDマウント/チェックフラグ(lower byte)
#define EEP_W_MNT_FLAGS 71  // 各種マウントフラグ
#define EEP_W_EXSERIAL_BPS  72  // 拡張シリアル速度(/100)
#define EEP_W_EXSERIAL_TMO  73  // 拡張シリアルタイムアウト
#define EEP_W_PC_BPS    74  // PCシリアル速度(/100)
#define EEP_W_PC_TMOUT  75  // PCシリアルタイムアウト
#define EEP_W_SV_TYPE_LR 76 // サーボプロトコル(upper=L, lower=R)
#define EEP_W_SV_TYPE_CX 77 // サーボプロトコル(upper=C, lower=X)
#define EEP_W_SV_BPS_L  78  // サーボ速度L(/100)
#define EEP_W_SV_BPS_R  79  // サーボ速度R(/100)
#define EEP_W_SV_BPS_C  80  // サーボ速度C(/100)
#define EEP_W_SV_BPS_X  81  // サーボ速度X(/100)
#define EEP_W_SV_TMO_LR 82  // サーボタイムアウト(upper=L, lower=R)
#define EEP_W_SV_TMO_CX 83  // サーボタイムアウト(upper=C, lower=X)
#define EEP_W_SV_LOST   84  // サーボロスト判定フレーム数(lower byte)
#define EEP_W_OP_MODE   90  // 動作モードフラグ
#define EEP_W_IMU_TYPE  91  // IMU/AHRSタイプ(lower byte)
#define EEP_W_IMU_CAL   92  // IMUキャリブレーション値先頭(word 92-97)
#define EEP_W_IMU_INTV  98  // IMU読み取り間隔(upper), 移動平均数(lower)
#define EEP_W_C_SV      99  // C系サーボ先頭(word 99-108)
#define EEP_W_FRAME_MS  109 // フレーム時間(ms)
#define EEP_W_L_SV      110 // L系サーボ先頭(word 110-139, 15個×2word)
#define EEP_W_R_SV      140 // R系サーボ先頭(word 140-169, 15個×2word)
#define EEP_W_PAD_TYPE  175 // リモコンタイプ
#define EEP_W_PAD_DETAIL 176 // リモコン詳細(upper=timeout/1000, lower=interval)
#define EEP_W_MONITOR   178 // モニタリングフラグ(lower byte)
#define EEP_W_CRC       179 // CRC16(word 0-178)

// EEPROM初期化識別子 (word 0 の下位バイト)
#define EEP_INIT_ID 0x55

// EEPROM固定領域ワード数
#define EEP_FIXED_WORDS 180

// EEPROM読み書き用共用体 (EEPROM_SIZE = 540バイト = 270ワード)
typedef union {
  uint8_t  bval[EEPROM_SIZE];          // バイト単位アクセス
  int16_t  sval[EEPROM_SIZE / 2];      // ワード単位アクセス(符号付き)
  uint16_t usval[EEPROM_SIZE / 2];     // ワード単位アクセス(符号なし)
  int16_t  saval[3][90];               // 旧3行Meridim構造(後方互換)
  uint16_t usaval[3][90];              // 旧3行Meridim構造(後方互換, 符号なし)
} UnionEEPROM;

extern UnionEEPROM eeprom_write_data; // EEPROM書き込み用
extern UnionEEPROM eeprom_read_data;  // EEPROM読み込み用

//------------------------------------------------------------------------------------
//  ネットワーク設定ランタイムバッファ (EEPROM_LOAD=1 時にEEPROMから取得)
//------------------------------------------------------------------------------------
extern char     eep_ssid[33];      // WiFi SSID
extern char     eep_pass[65];      // WiFi PASS
extern char     eep_send_ip[16];   // 送信先PC IPアドレス
extern char     eep_fixed_ip[16];  // ESP32固定IPアドレス
extern char     eep_gateway[16];   // ゲートウェイIPアドレス
extern char     eep_subnet[16];    // サブネットマスク
extern uint16_t eep_send_port;     // UDP送信ポート
extern uint16_t eep_recv_port;     // UDP受信ポート
extern uint16_t eep_frame_ms;      // フレーム時間(ms)

//==================================================================================================
//  関数プロトタイプ宣言
//==================================================================================================

/// @brief EEPROMの初期化
/// @param a_eeprom_size EEPROMのバイト長
/// @return 初期化が成功すればtrue, 失敗ならfalseを返す.
bool mrd_eeprom_init(int a_eeprom_size);

/// @brief config.h の設定値からEEPROM格納用の配列データを作成する.
/// @param a_sv サーボ設定を保持する構造体.
/// @return EEPROM格納用の配列データ(UnionEEPROM型).
UnionEEPROM mrd_eeprom_make_data_from_config(const ServoParam &a_sv);

/// @brief EEPROMの内容を読み込んで返す.
/// @return UnionEEPROM のフォーマットで配列を返す.
UnionEEPROM mrd_eeprom_read();

/// @brief EEPROMの内容を読み込みランタイム設定に反映する.
///        ネットワーク・サーボ・IMU/フレーム時間を適用する.
/// @param a_sv サーボ設定を保持する構造体.
/// @param a_serial 出力先シリアルの指定.
/// @return 成功時にtrue, CRC不一致などの場合はfalseを返す.
bool mrd_eeprom_load_config(ServoParam &a_sv, HardwareSerial &a_serial);

/// @brief EEPROM格納用の配列データをシリアルにダンプ出力する.
/// @param a_data EEPROM用の配列データ.
/// @param a_bhd ダンプリストの表示形式.(0:Bin, 1:Hex, 2:Dec)
/// @return 終了時にtrueを返す.
bool mrd_eeprom_dump_to_serial(UnionEEPROM a_data, int a_bhd, HardwareSerial &a_serial);

/// @brief EEPROM格納用の配列データをシリアルにダンプ出力する.(起動時用)
/// @param a_do_dump 実施するか否か.
/// @param a_bhd ダンプリストの表示形式.(0:Bin, 1:Hex, 2:Dec)
/// @return 終了時にtrueを返す.
bool mrd_eeprom_dump_at_boot(bool a_do_dump, int a_bhd, HardwareSerial &a_serial);

/// @brief EEPROMにEEPROM格納用の配列データを書き込む.
/// @param a_write_data EEPROM書き込み用の配列データ.
/// @param a_flg_protect EEPROMの書き込み許可があるかどうかのブール値.
/// @return EEPROMの書き込みと読み込みが成功した場合はtrueを, 書き込まなかった場合はfalseを返す.
bool mrd_eeprom_write(UnionEEPROM a_write_data, bool a_flg_protect, HardwareSerial &a_serial);

/// @brief EEPROMに設定値を書き込み, その後で読み込んで内容を確認し, シリアルポートに出力する.
/// @param a_write_data EEPROM書き込み用の配列データ.
/// @param a_do EEPROMの読み書きチェックを実施するかのブール値.
/// @param a_protect EEPROMの書き込み許可があるかどうかのブール値.
/// @param a_bhd ダンプリストの表示形式.(0:Bin, 1:Hex, 2:Dec)
/// @return EEPROMの書き込みと読み込みが成功した場合はtrueを, それ以外はfalseを返す.
bool mrd_eeprom_write_read_check(UnionEEPROM a_write_data, bool a_do, bool a_protect, int a_bhd);

/// @brief EEPROMから任意のshort型データを読み込む(旧インデックス形式).
/// @param index_y 配列の一次元目(0~2).
/// @param index_x 配列の二次元目(0~89).
/// @return short型データを返す.
short mrd_eeprom_read_short(int index_y, int index_x);

/// @brief EEPROMから任意のbyte型データを読み込む(旧インデックス形式).
/// @param index_y 配列の一次元目(0~2).
/// @param index_x 配列の二次元目(0~179).
/// @param low_high 下位ビットか上位ビットか. (0:low_bit, 1:high_bit)
/// @return byte型データを返す.
int8_t mrd_eeprom_read_byte(int index_y, int index_x, int low_high);

#endif // __MERIDIAN_EEPROM_H__
