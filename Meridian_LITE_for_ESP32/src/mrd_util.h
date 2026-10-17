#ifndef __MERIDIAN_UTILITY_H__
#define __MERIDIAN_UTILITY_H__

// ヘッダファイルの読み込み
#include "config.h"
#include "main.h"

//==================================================================================================
// Utility ごく小規模な汎用関数
//==================================================================================================

/// @brief 配列の中で0以外が入っている最大のIndexに1を足した数(使っている長さ)を求める.
/// @param a_arr 配列
/// @param a_size 配列の長さ
/// @return 0以外が入っている最大のIndex + 1. すべて0の場合は1を返す.
int mrd_index_max_used_get(const int a_arr[], int a_size) {
  int max_index_tmp = 0;
  for (int i = 0; i < a_size; ++i) {
    if (a_arr[i] != 0) {
      max_index_tmp = i;
    }
  }
  return max_index_tmp + 1;
}

/// @brief 指定された位置のビットをセットする(16ビット変数用).
/// @param a_byte ビットをセットする16ビットの変数.参照渡し.
/// @param a_bit_pos セットするビットの位置(0から15).
/// @return なし.
inline void mrd_bit16_set(uint16_t &a_byte, uint16_t a_bit_pos) { a_byte |= (1 << a_bit_pos); }

/// @brief 指定された位置のビットをクリアする(16ビット変数用).
/// @param a_byte ビットをクリアする16ビットの変数.参照渡し.
/// @param a_bit_pos クリアするビットの位置(0から15).
/// @return なし.
inline void mrd_bit16_clear(uint16_t &a_byte, uint16_t a_bit_pos) { a_byte &= ~(1 << a_bit_pos); }

//------------------------------------------------------------------------------------
//  表示用
//------------------------------------------------------------------------------------

/// @brief 数値をシリアルモニタ表示するときにパディングする.
/// @param a_num 表示したい値.
/// @param a_total_width 桁数.
/// @param a_frac_width 小数点以下の桁数(0ならば小数点以下非表示).
/// @param a_show_plus +記号の有無.
/// @return 整形されたストリング.
String mrd_pddstr(float a_num, int a_total_width, int a_frac_width, bool a_show_plus = true) {
  char buf_tmp[32];
  snprintf(buf_tmp, sizeof(buf_tmp), a_show_plus ? "%+*.*f" : "%*.*f", a_total_width, a_frac_width, double(a_num));
  return String(buf_tmp);
}

/// @brief 列挙型(L,R,C)から文字列を取得する関数.
/// @param a_line 列挙型 enum UartLine
/// @return 列挙型の内容に応じて文字列"L","R","C"返す.
const char *mrd_line_name_get(UartLine a_line) {
  switch (a_line) {
  case L:
    return "L";
  case R:
    return "R";
  case C:
    return "C";
  default:
    return "Unknown";
  }
}

//------------------------------------------------------------------------------------
//  その他
//------------------------------------------------------------------------------------

/// @brief エラーメッセージ＆LED点滅と共にシステム停止.
/// @param a_led ERROR通知用のLEDのピン番号.
/// @param a_msg エラーメッセージ.
/// @param a_serial 出力先シリアルの指定.
void mrd_error_stop(int a_led, String a_msg, HardwareSerial &a_serial) {
  a_serial.println(a_msg);
  while (1) {
    mrd_led_write(a_led, HIGH);
    delay(250);
    mrd_led_write(a_led, LOW);
    delay(250);
  }
}

/// @brief 目標値の変化量を基準値から±a_step_maxの範囲に制限する.
/// @param a_tgt 目標値.
/// @param a_base 基準値(前回の指令値など).
/// @param a_step_max 最大変化量.
/// @return 制限後の目標値.
float mrd_step_limit(float a_tgt, float a_base, float a_step_max) {
  return constrain(a_tgt, a_base - a_step_max, a_base + a_step_max);
}

#endif //__MERIDIAN_UTILITY_H__
