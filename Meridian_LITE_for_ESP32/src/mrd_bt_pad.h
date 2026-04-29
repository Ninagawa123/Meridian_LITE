#ifndef __MERIDIAN_BT_PAD_H__
#define __MERIDIAN_BT_PAD_H__

// ヘッダファイルの読み込み
#include "config.h"
#include <Arduino.h>

// ライブラリ導入
#include <ESP32Wiimote.h> // Wiiコントローラー
extern ESP32Wiimote wiimote;

#include <IcsHardSerialClass.h>
extern PadUnion pad_array;
extern SemaphoreHandle_t pad_mutex; // pad_arrayアクセス用Mutex

// リモコンボタンデータ変換テーブル
constexpr unsigned short PAD_TABLE_WIIMOTE_SOLO[16] = {
    0x1000, 0x0080, 0x0000, 0x0010, 0x0200, 0x0400, 0x0100, 0x0800,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0008, 0x0001, 0x0002, 0x0004};
constexpr unsigned short PAD_TABLE_WIIMOTE_ORIG[16] = {
    0x0100, 0x0200, 0x0400, 0x0800, 0x1000, 0x0000, 0x0000, 0x0000,
    0x0001, 0x0002, 0x0004, 0x0008, 0x0010, 0x0000, 0x0000, 0x0080};
constexpr unsigned short PAD_TABLE_KRC5FH_TO_COMMON[16] = { //
    0, 64, 32, 128, 1, 4, 2, 8, 1024, 4096, 512, 2048, 16, 64, 32, 256};

// KRC-5FH 十字キーコンボ検出用ボタンマスク
constexpr uint16_t KRC_DPAD_LEFT_MASK = 0x000F;   // bit 0-3: 左十字キー (UP,DOWN,LEFT,RIGHT)
constexpr uint16_t KRC_DPAD_RIGHT_MASK = 0x0170;  // bit 4,5,6,8: 右十字キーボタン (368)
constexpr uint16_t KRC_CLEAR_DPAD_LEFT = 0xFFF0;  // bit 0-3 をクリア
constexpr uint16_t KRC_CLEAR_DPAD_RIGHT = 0xFE8F; // bit 4,5,6,8 をクリア
// ボタンテーブル変換から残った可能性のあるbit 1,2 をクリア
constexpr uint16_t KRC_CLEAR_RESIDUAL_BITS = 0xFFF9; // 0b1111111111111001

//==================================================================================================
//  JOYPAD 関数宣言
//==================================================================================================

//----------------------------------------------------------------------
// KRC-5FH 読み取り
//----------------------------------------------------------------------

/// @brief KRC-5FHジョイパッドからデータを読み取り, 指定間隔で更新する
/// @param a_interval 読み取り間隔 (ミリ秒)
/// @param a_ics ICSシリアルクラスインスタンス
/// @return 更新されたジョイパッド状態を64ビット整数で返す
uint64_t mrd_pad_read_krc(uint a_interval, IcsHardSerialClass &a_ics);

//----------------------------------------------------------------------
// WIIMOTE 読み取り
//----------------------------------------------------------------------

/// @brief Wiiリモコンからの入力データを受信・処理する
/// @return 更新されたジョイパッド状態を64ビット整数で返す
/// @note 内部でESP32Wiimoteインスタンスwiimoteと定数PAD_GENERALIZEを使用
uint64_t mrd_bt_read_wiimote();

//==================================================================================================
//  各種パッドへの分岐
//==================================================================================================

/// @brief 指定されたジョイパッドタイプに基づいて最新データを読み取り, 64ビット整数で返す
/// @param a_pad_type ジョイパッドタイプのenum (MERIMOTE, BLUERETRO, SBDBT, KRR5FH)
/// @param a_pad_data 64ビットボタンデータ
/// @param a_ics ICSシリアルクラスインスタンス (KRR5FH使用時)
/// @return 受信データを64ビット整数に変換したもの
/// @note WIIMOTEの場合, スレッドが自動的にpad_array.ui64valを更新
uint64_t mrd_pad_read(PadType a_pad_type, uint64_t a_pad_data, IcsHardSerialClass &a_ics);

//==================================================================================================
//  初期化とセットアップ
//==================================================================================================

//----------------------------------------------------------------------
// Bluetooth, WIIMOTE 初期化
//----------------------------------------------------------------------

/// @brief Bluetoothを設定し, Wiiコントローラー接続を開始する
/// @param a_mount_pad 搭載パッドタイプ
/// @param a_timeout 接続タイムアウト
/// @param a_wiimote Wiimoteインスタンス
/// @param a_led LEDピン
/// @param a_serial 出力シリアル
/// @return 成功時はtrue, タイムアウト時はfalse
bool mrd_bt_settings(int a_mount_pad,
                     int a_timeout,
                     ESP32Wiimote &a_wiimote,
                     int a_led,
                     HardwareSerial &a_serial);

//----------------------------------------------------------------------
// WIIMOTE スレッド
//----------------------------------------------------------------------

/// @brief サブCPU (Core0) で実行されるBluetooth通信ルーチン
/// @param args この関数に渡される引数. 現在は未使用.
/// @note 内部でPadUnion型pad_array.ui64valと定数PAD_INTERVAL, WIIMOTEを使用
void Core0_BT_r(void *args);

//------------------------------------------------------------------------------------
//  meridimデータ書き込み
//------------------------------------------------------------------------------------

/// @brief PADデータをmeridim配列に書き込む
/// @param a_meridim Meridim配列共用体. 参照渡し.
/// @param a_pad_array PAD受信値用配列
/// @param a_marge PADボタンデータのマージ用ブール値.
///        true: 既存データとビット論理和, false: 新データで上書き.
/// @return 完了時にtrueを返す
bool meriput90_pad(Meridim90Union &a_meridim, PadUnion a_pad_array, bool a_marge);

#endif // __MERIDIAN_BT_PAD_H__
