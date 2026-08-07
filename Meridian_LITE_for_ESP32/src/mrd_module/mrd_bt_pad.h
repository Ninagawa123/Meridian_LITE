#ifndef __MERIDIAN_BT_PAD_H__
#define __MERIDIAN_BT_PAD_H__

// ヘッダファイルの読み込み
#include "mrd_types.h"

// ライブラリ導入
#include <IcsHardSerialClass.h> // ICSサーボのインスタンス設定
#include <ESP32Wiimote.h>       // Wiiコントローラー

// ics_L, ics_R は mrd_types.h で extern 宣言済み

// グローバル変数の宣言
extern ESP32Wiimote wiimote;
extern PadUnion pad_array; // pad値の格納用配列

// リモコン受信ボタンデータの変換テーブル
constexpr unsigned short PAD_TABLE_WIIMOTE_SOLO[16] = {
    0x1000, 0x0080, 0x0000, 0x0010, 0x0200, 0x0400, 0x0100, 0x0800,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0008, 0x0001, 0x0002, 0x0004};
constexpr unsigned short PAD_TABLE_WIIMOTE_ORIG[16] = {
    0x0100, 0x0200, 0x0400, 0x0800, 0x1000, 0x0000, 0x0000, 0x0000,
    0x0001, 0x0002, 0x0004, 0x0008, 0x0010, 0x0000, 0x0000, 0x0080};
constexpr unsigned short PAD_TABLE_KRC5FH_TO_COMMON[16] = { //
    0, 64, 32, 128, 1, 4, 2, 8, 1024, 4096, 512, 2048, 16, 64, 32, 256};

//==================================================================================================
//  関数プロトタイプ宣言
//==================================================================================================

/// @brief KRC-5FHジョイパッドからデータを読み取り, 指定された間隔でデータを更新する.
/// @param a_interval 読み取り間隔(ミリ秒).
/// @return 更新されたジョイパッドの状態を64ビット整数で返す.
uint64_t mrd_pad_read_krc(uint a_interval, IcsHardSerialClass &a_ics);

/// @brief Wiiリモコンからの入力データを受信し, 処理する.
/// @return 更新されたジョイパッドの状態を64ビット整数で返す.
/// @note ESP32Wiimoteインスタンス wiimote, 定数PAD_GENERALIZE を関数内で使用.
uint64_t mrd_bt_read_wiimote();

/// @brief 指定されたジョイパッドタイプに応じて最新データを読み取り, 64ビット整数で返す.
/// @param a_pad_type ジョイパッドのタイプを示す列挙型(MERIMOTE, BLUERETRO, SBDBT, KRR5FH).
/// @param a_pad_data 64ビットのボタンデータ
/// @return 64ビット整数に変換された受信データ
/// @note WIIMOTEの場合は, スレッドがpad_array.ui64valを自動更新.
uint64_t mrd_pad_read(PadType a_pad_type, uint64_t a_pad_data);

/// @brief Bluetoothの設定を行い, Wiiコントローラの接続を開始する.
bool mrd_bt_settings(int a_mount_pad,
                     int a_timeout,
                     ESP32Wiimote &a_wiimote,
                     int a_led,
                     HardwareSerial &a_serial);

/// @brief サブCPU (Core0) で実行されるBluetooth通信用のルーチン.
/// @param args この関数に渡される引数. 現在は不使用.
/// @note PadUnion型の pad_array.ui64val, 定数PAD_INTERVAL, WIIMOTE を関数内で使用.
void Core0_BT_r(void *args);

/// @brief meridim配列にPADデータを書き込む.
/// @param a_meridim Meridim配列の共用体. 参照渡し.
/// @param a_pad_array PAD受信値の格納用配列.
/// @param a_marge PADボタンデータをマージするかどうかのブール値.
/// trueの場合は既存のデータにビット単位でOR演算を行い, falseの場合は新しいデータで上書きする.
bool meriput90_pad(Meridim90Union &a_meridim, PadUnion a_pad_array, bool a_marge);

#endif // __MERIDIAN_BT_PAD_H__
