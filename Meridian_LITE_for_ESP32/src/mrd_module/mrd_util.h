#ifndef __MERIDIAN_UTIL_H__
#define __MERIDIAN_UTIL_H__

// ヘッダファイルの読み込み
#include "../config.h"
#include "mrd_types.h"
#include <Arduino.h>

//==================================================================================================
// 関数プロトタイプ宣言
//==================================================================================================

/// @brief 配列の中で0以外が入っている最大のIndexを求める.
/// @param a_arr 配列
/// @param a_size 配列の長さ
/// @return 0以外が入っている最大のIndex. すべて0の場合は1を反す.
int mrd_max_used_index(const int a_arr[], int a_size);

/// @brief 指定された位置のビットをセットする(16ビット変数用).
/// @param a_byte ビットをセットする16ビットの変数.参照渡し.
/// @param a_bit_pos セットするビットの位置(0から15).
/// @return なし.
inline void mrd_set_bit16(uint16_t &a_byte, uint16_t a_bit_pos) { a_byte |= (1 << a_bit_pos); }

/// @brief 指定された位置のビットをクリアする(16ビット変数用).
/// @param a_byte ビットをクリアする16ビットの変数.参照渡し.
/// @param a_bit_pos クリアするビットの位置(0から15).
/// @return なし.
inline void mrd_clear_bit16(uint16_t &a_byte, uint16_t a_bit_pos) { a_byte &= ~(1 << a_bit_pos); }

/// @brief 指定された位置のビットをセットする(8ビット変数用).
/// @param value ビットをセットする8ビットの変数.参照渡し.
/// @param a_bit_pos セットするビットの位置(0から7).
/// @return なし.
inline void mrd_set_bit8(uint8_t &value, uint8_t a_bit_pos) { value |= (1 << a_bit_pos); }

/// @brief 指定された位置のビットをクリアする(8ビット変数用).
/// @param value ビットをクリアする8ビットの変数.参照渡し.
/// @param a_bit_pos クリアするビットの位置(0から7).
/// @return なし.
inline void mrd_clear_bit8(uint8_t &value, uint8_t a_bit_pos) { value &= ~(1 << a_bit_pos); }

/// @brief 任意の整数値から、任意幅のビット列を取り出す汎用関数.
/// @tparam T 型テンプレート.任意の整数型(符号付き・符号無しどちらでも可)
/// @param value 抽出元となる値.
/// @param pos   取り出し開始位置(LSB＝0, 右から数え, 最初は0番)
/// @param len   取り出すビット幅
/// @return unsigned 取り出したビット列(0〜 2^len−1 の範囲)
template <class T> // 型テンプレート
unsigned mrd_slice_bits(T value, unsigned pos, unsigned len) {
  return (static_cast<unsigned>(value) >> pos) & ((1u << len) - 1u);
}

//------------------------------------------------------------------------------------
//  表示用
//------------------------------------------------------------------------------------

/// @brief 数値をシリアルモニタ表示するときにパディングする.
/// @param num 表示したい値.
/// @param total_width 桁数.
/// @param frac_width 小数点以下の桁数(0ならば小数点以下非表示).
/// @param show_plus +記号の有無.
/// @return 整形された文字列(static buffer).
const char *mrd_pddstr(float num, int total_width, int frac_width, bool show_plus = true);

//------------------------------------------------------------------------------------
//  タイムアウト監視用タイマー
//------------------------------------------------------------------------------------

/// @brief 指定されたミリ秒のタイムアウトを監視する. mrd_timeout_resetとセットで使う.
/// @param a_limit タイムアウトまでの時間(ms)
/// @return タイムアウトでtrueを返す.
bool mrd_timeout_check(unsigned long a_limit);

/// @brief タイムアウト監視開始フラグをリセットする. mrd_timeout_checkとセットで使う.
void mrd_timeout_reset();

/// @brief 列挙型(L,R,C)から文字列を取得する関数.
/// @param a_line 列挙型 enum UartLine
/// @return 列挙型の内容に応じて文字列"L","R","C"返す.
const char *mrd_get_line_name(UartLine a_line);

//------------------------------------------------------------------------------------
//  meriput / meridimへのデータ書き込み
//------------------------------------------------------------------------------------

/// @brief meridim配列のチェックサムを算出して[len-1]に書き込む.
/// @param a_meridim Meridim配列の共用体. 参照渡し.
/// @return 常にtrueを返す.
bool mrd_meriput90_cksm(Meridim90Union &a_meridim, int len = 90);

/// @brief 前回のシーケンス番号から次の予測値を返す. 範囲は 0〜59999.
/// @param a_previous_num 前回のシーケンス番号.
/// @return 次のシーケンス番号.
inline uint16_t mrd_seq_predict_num(uint16_t a_previous_num) {
  uint16_t x_tmp = a_previous_num + 1;
  if (x_tmp > 59999) {
    x_tmp = 0;
  }
  return x_tmp;
}

//------------------------------------------------------------------------------------
//  CRC
//------------------------------------------------------------------------------------

/// @brief CRC-16/CCITT チェックサムを計算する. EEPROM整合性チェック等に使用.
/// @param a_data チェック対象の uint16_t 配列.
/// @param a_len  配列の要素数(word数).
/// @return 計算されたCRC-16値.
uint16_t mrd_crc16(const uint16_t *a_data, int a_len);

#endif // __MERIDIAN_UTIL_H__
