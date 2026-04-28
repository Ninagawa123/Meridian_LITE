#ifndef __MERIDIAN_UTILITY_H__
#define __MERIDIAN_UTILITY_H__

// ヘッダファイルの読み込み
#include <Arduino.h>
#include <IPAddress.h>
#include "mrd_common.h"

//==================================================================================================
// ユーティリティ関数宣言
//==================================================================================================

/// @brief 配列内で非ゼロ値を持つ最大インデックス+1を取得する (ループ回数用)
/// @param a_arr 配列
/// @param a_size 配列の長さ
/// @return 非ゼロ値を持つ最大インデックス+1. 全てゼロの場合は1を返す.
int mrd_max_used_index(const int a_arr[], int a_size);

/// @brief 指定位置のビットをセットする (16ビット変数)
/// @param a_byte ビットをセットする16ビット変数. 参照渡し.
/// @param a_bit_pos セットするビット位置 (0から15)
/// @return void
inline void mrd_set_bit16(uint16_t &a_byte, uint16_t a_bit_pos) { a_byte |= (1 << a_bit_pos); }

/// @brief 指定位置のビットをクリアする (16ビット変数)
/// @param a_byte ビットをクリアする16ビット変数. 参照渡し.
/// @param a_bit_pos クリアするビット位置 (0から15)
/// @return void
inline void mrd_clear_bit16(uint16_t &a_byte, uint16_t a_bit_pos) { a_byte &= ~(1 << a_bit_pos); }

/// @brief 指定位置のビットをセットする (8ビット変数)
/// @param value ビットをセットする8ビット変数. 参照渡し.
/// @param a_bit_pos セットするビット位置 (0から7)
/// @return void
inline void mrd_set_bit8(uint8_t &value, uint8_t a_bit_pos) { value |= (1 << a_bit_pos); }

/// @brief 指定位置のビットをクリアする (8ビット変数)
/// @param value ビットをクリアする8ビット変数. 参照渡し.
/// @param a_bit_pos クリアするビット位置 (0から7)
/// @return void
inline void mrd_clear_bit8(uint8_t &value, uint8_t a_bit_pos) { value &= ~(1 << a_bit_pos); }

/// @brief 整数から任意幅のビットフィールドを抽出する汎用関数
/// @tparam T 型テンプレート. 任意の整数型 (符号付きまたは符号なし)
/// @param value 抽出元の値
/// @param pos 抽出開始位置 (LSB=0, 右から数えて0開始)
/// @param len 抽出するビット幅
/// @return unsigned 抽出されたビットフィールド (範囲 0 から 2^len-1)
template <class T> // 型テンプレート
unsigned mrd_slice_bits(T value, unsigned pos, unsigned len) {
  return (static_cast<unsigned>(value) >> pos) & ((1u << len) - 1u);
}

//------------------------------------------------------------------------------------
//  表示フォーマット
//------------------------------------------------------------------------------------

/// @brief シリアルモニタ表示用に数値をパディングする
/// @param num 表示する値
/// @param total_width 桁数
/// @param frac_width 小数点以下の桁数 (0で小数点なし)
/// @param show_plus +符号を表示するか (デフォルトtrue)
/// @return フォーマット済み文字列
String mrd_pddstr(float num, int total_width, int frac_width, bool show_plus = true);

//------------------------------------------------------------------------------------
//  タイムアウト監視タイマー
//------------------------------------------------------------------------------------

/// @brief 指定ミリ秒のタイムアウトを監視する. mrd_timeout_resetと併用.
/// @param a_limit タイムアウト時間 (ms)
/// @return タイムアウト時はtrue
bool mrd_timeout_check(unsigned long a_limit);

/// @brief タイムアウト監視フラグをリセットする. mrd_timeout_checkと併用.
void mrd_timeout_reset();

/// @brief enum (L, R, C) から文字列を取得する
/// @param a_line enum型 enum UartLine
/// @return enumの内容に応じて "L", "R", "C" の文字列
const char *mrd_get_line_name(UartLine a_line);

//------------------------------------------------------------------------------------
//  meriput / meridimデータ書き込み
//------------------------------------------------------------------------------------

/// @brief エラーメッセージとLED点滅, タイムアウト後に再起動
/// @param a_led エラー通知LED用のピン番号
/// @param a_msg エラーメッセージ
/// @param a_serial 出力シリアル
/// @param a_restart_ms 再起動までのタイムアウト (ms, デフォルト10000ms, 0で再起動しない)
void mrd_error_stop(int a_led, String a_msg, HardwareSerial &a_serial, unsigned long a_restart_ms = 10000);

/// @brief meridim配列のチェックサムを計算して[len-1]に書き込む
/// @param a_meridim Meridim配列共用体. 参照渡し.
/// @param len 配列の長さ (デフォルト90)
/// @return 常にtrueを返す
bool mrd_meriput90_cksm(Meridim90Union &a_meridim, int len = 90);

//------------------------------------------------------------------------------------
//  ネットワークヘルパー関数 (WiFi/Ethernet共通)
//------------------------------------------------------------------------------------

/// @brief IPアドレス設定を検証する
/// @param local_ip ローカルIP
/// @param gateway ゲートウェイIP
/// @param subnet サブネットマスク
/// @param a_serial エラー出力用シリアル
/// @return 検証結果 (true: OK, false: NG)
bool mrd_validate_network_config(IPAddress local_ip, IPAddress gateway, IPAddress subnet, HardwareSerial &a_serial);

/// @brief IPアドレス文字列をIPAddressオブジェクトに変換する
/// @param ip_str IPアドレス文字列 (例: "192.168.1.1")
/// @param a_serial エラー出力用シリアル
/// @return 成功時はIPAddressオブジェクト, 失敗時はIPAddress(0,0,0,0)
IPAddress mrd_parse_ip_address(const char *ip_str, HardwareSerial &a_serial);

#endif //__MERIDIAN_UTILITY_H__
