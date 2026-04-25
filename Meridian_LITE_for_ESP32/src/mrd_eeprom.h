#ifndef __MERIDIAN_EEPROM_H__
#define __MERIDIAN_EEPROM_H__

// ヘッダファイルの読み込み
#include "config.h"
#include "main.h"
#include "mrd_util.h"

// ライブラリ導入
#include <EEPROM.h>

// EEPROM読み書き用共用体
typedef union {
  uint8_t bval[EEPROM_SIZE]; // 1バイト単位で540個のデータを持つ
  int16_t saval[3][90];      // short型で3*90個の配列データを持つ
  uint16_t usaval[3][90];    // unsigned short型で3*90個の配列データを持つ
  int16_t sval[270];         // short型で270個のデータを持つ
  uint16_t usval[270];       // unsigned short型で270個のデータを持つ
} UnionEEPROM;
UnionEEPROM eeprom_write_data; // EEPROM書き込み用
UnionEEPROM eeprom_read_data;  // EEPROM読み込み用

//==================================================================================================
//  EEPROM関連の処理
//==================================================================================================

/// @brief EEPROMの初期化
/// @param a_eeprom_size EEPROMのバイト長
/// @return 初期化が成功すればtrue, 失敗ならfalseを返す.
bool mrd_eeprom_init(int a_eeprom_size) {
  if (EEPROM.begin(a_eeprom_size)) {
    return true;
  } else {
  }
  return false;
}

/// @brief サーボ設定構造体からEEPROM格納用の配列データを作成する
/// @param a_sv サーボ設定を保持する構造体
/// @return EEPROM格納用の配列データ(UnionEEPROM型)
UnionEEPROM mrd_eeprom_make_data_from_config(const ServoParam &a_sv) {
  UnionEEPROM array_tmp = {0};

  for (int i = 0; i < 15; i++) {
    // 各サーボのマウント有無と方向(正転・逆転)
    uint16_t l_tmp = 0;
    uint16_t r_tmp = 0;

    // bit0 : マウント
    if (sv.ixl_mount[i])
      l_tmp |= 0x0001;
    if (sv.ixr_mount[i])
      r_tmp |= 0x0001;

    // bit1-7 : サーボ ID
    l_tmp |= static_cast<uint16_t>(sv.ixl_id[i] & 0x7F) << 1;
    r_tmp |= static_cast<uint16_t>(sv.ixr_id[i] & 0x7F) << 1;

    // bit8 : サーボ回転方向
    if (sv.ixl_cw[i] > 0)
      l_tmp |= 0x0100;
    if (sv.ixr_cw[i] > 0)
      r_tmp |= 0x0100;

    // サーボのマウント有無, ID, 回転方向のデータ格納
    array_tmp.saval[1][20 + i * 2] = l_tmp;
    array_tmp.saval[1][50 + i * 2] = r_tmp;

    // 各サーボの直立デフォルト角度(degree → float short*100)の格納
    array_tmp.saval[1][21 + i * 2] = mrd.float2HfShort(a_sv.ixl_trim[i]);
    array_tmp.saval[1][51 + i * 2] = mrd.float2HfShort(a_sv.ixr_trim[i]);
  }
  return array_tmp;
}

/// @brief EEPROMの内容を読み込んで返す.
/// @return UnionEEPROM のフォーマットで配列を返す.
UnionEEPROM mrd_eeprom_read() {
  UnionEEPROM read_data_tmp = {0};      // ゼロ初期化を明示的に行う
  for (int i = 0; i < EEPROM_SIZE; i++) // データを読み込む時はbyte型
  {
    read_data_tmp.bval[i] = EEPROM.read(i);
  }
  return read_data_tmp;
}

/// @brief EEPROMの内容を読み込みサーボ値構造体に反映する.
/// @param a_sv サーボ設定を保持する構造体.
/// @param a_monitor シリアルモニタへのデータ表示.
/// @param a_serial 出力先シリアルの指定.
/// @return 終了時にtrueを返す.
bool mrd_eeprom_load_servosettings(ServoParam &a_sv, bool a_monitor, HardwareSerial &a_serial) {
  a_serial.println("Load and set servo settings from EEPROM.");
  UnionEEPROM array_tmp = mrd_eeprom_read();
  for (int i = 0; i < a_sv.num_max; i++) {
    // 各サーボのマウント有無
    a_sv.ixl_mount[i] = static_cast<bool>(array_tmp.saval[1][20 + i * 2] & 0x0001); // bit0:マウント有無
    a_sv.ixr_mount[i] = static_cast<bool>(array_tmp.saval[1][50 + i * 2] & 0x0001); // bit0:マウント有無
    // 各サーボの実サーボ呼び出しID番号
    a_sv.ixl_id[i] = static_cast<uint8_t>(array_tmp.saval[1][20 + i * 2] >> 1 & 0x007F); // bit1–7:サーボID
    a_sv.ixr_id[i] = static_cast<uint8_t>(array_tmp.saval[1][50 + i * 2] >> 1 & 0x007F); // bit1–7:サーボID
    // 各サーボの回転方向(正転・逆転)
    a_sv.ixl_cw[i] = static_cast<int8_t>((array_tmp.saval[1][20 + i * 2] >> 8) & 0x0001) ? 1 : -1; // bit8:回転方向
    a_sv.ixr_cw[i] = static_cast<int8_t>((array_tmp.saval[1][50 + i * 2] >> 8) & 0x0001) ? 1 : -1; // bit8:回転方向
    // 各サーボの直立デフォルト角度,トリム値(degree小数2桁までを100倍した値で格納されているものを展開)
    a_sv.ixl_trim[i] = array_tmp.saval[1][21 + i * 2] / 100.0f;
    a_sv.ixr_trim[i] = array_tmp.saval[1][51 + i * 2] / 100.0f;

    if (a_monitor) {
      a_serial.print("L-idx:");
      a_serial.print(mrd_pddstr(i, 2, 0, false));
      a_serial.print(", id:");
      a_serial.print(mrd_pddstr(sv.ixl_id[i], 2, 0, false));
      a_serial.print(", mt:");
      a_serial.print(mrd_pddstr(sv.ixl_mount[i], 1, 0, false));
      a_serial.print(", cw:");
      a_serial.print(mrd_pddstr(sv.ixl_cw[i], 1, 0, true));
      a_serial.print(", trm:");
      a_serial.print(mrd_pddstr(sv.ixl_trim[i], 7, 2, true));
      a_serial.print("  R-idx: ");
      a_serial.print(mrd_pddstr(i, 2, 0, false));
      a_serial.print(", id:");
      a_serial.print(mrd_pddstr(sv.ixr_id[i], 2, 0, false));
      a_serial.print(", mt:");
      a_serial.print(mrd_pddstr(sv.ixr_mount[i], 1, 0, false));
      a_serial.print(", cw:");
      a_serial.print(mrd_pddstr(sv.ixr_cw[i], 1, 0, true));
      a_serial.print(", trm:");
      a_serial.println(mrd_pddstr(sv.ixr_trim[i], 7, 2, true));
    }
  }
  return true;
}

/// @brief EEPROM格納用の配列データをシリアルにダンプ出力する.
/// @param a_data EEPROM用の配列データ.
/// @param a_bhd ダンプリストの表示形式.(0:Bin, 1:Hex, 2:Dec)
/// @return 終了時にtrueを返す.
bool mrd_eeprom_dump_to_serial(UnionEEPROM a_data, int a_bhd, HardwareSerial &a_serial) {
  int len_tmp = EEPROM.length(); // EEPROMの長さ
  a_serial.print("EEPROM Length ");
  a_serial.print(len_tmp); // EEPROMの長さ表示
  a_serial.println("byte, 16bit Dump;");
  for (int i = 0; i < 270; i++) // 読み込むデータはshort型で作成
  {
    if (a_bhd == 0) {
      a_serial.print(a_data.sval[i], BIN);
    } else if (a_bhd == 1) {
      a_serial.print(a_data.sval[i], HEX);
    } else {
      a_serial.print(a_data.sval[i], DEC);
    }
    if ((i == 89) or (i == 179) or (i == 269)) {
      a_serial.println();
    } else {
      a_serial.print("/");
    }
  }
  return true;
}

// /// @brief EEPROM格納用の配列データをシリアルにダンプ出力する.(起動時用)
// /// @param a_do_dump 実施するか否か.
// /// @param a_bhd ダンプリストの表示形式.(0:Bin, 1:Hex, 2:Dec)
// /// @return 終了時にtrueを返す.
// bool mrd_eeprom_show_at_boot(bool a_do_dump, int a_bhd, HardwareSerial &a_serial) {
//   if (a_do_dump) {
//     mrd_eeprom_dump_to_serial(mrd_eeprom_read(), a_bhd, a_serial);
//     return true;
//   }
//   return false;
// }

/// @brief EEPROM格納用の配列データをシリアルにダンプ出力する.(起動時用)
/// @param a_do_dump 実施するか否か.
/// @param a_bhd ダンプリストの表示形式.(0:Bin, 1:Hex, 2:Dec)
/// @return 終了時にtrueを返す.
bool mrd_eeprom_dump_at_boot(bool a_do_dump, int a_bhd, HardwareSerial &a_serial) {
  if (a_do_dump) {
    mrd_eeprom_dump_to_serial(mrd_eeprom_read(), a_bhd, a_serial);
    return true;
  }
  return false;
}

/// @brief EEPROMにEEPROM格納用の配列データを書き込む.
/// @param a_write_data EEPROM書き込み用の配列データ.
/// @param a_flg_protect EEPROMの書き込み許可があるかどうかのブール値.
/// @return EEPROMの書き込みと読み込みが成功した場合はtrueを, 書き込まなかった場合はfalseを返す.
bool mrd_eeprom_write(UnionEEPROM a_write_data, bool a_flg_protect, HardwareSerial &a_serial) {
  if (a_flg_protect) { // EEPROM書き込み実施フラグをチェック
    return false;
  }
  if (flg.eeprom_protect) // config.hのEEPROM書き込みプロテクトをチェック
  {
    Serial.println("EEPROM is protected. To unprotect, please set 'EEPROM_PROTECT' to false.");
    return false;
  }

  // EEPROM書き込み
  byte old_value_tmp;                   // EEPROMにすでに書き込んであるデータ
  bool flg_renew_tmp = false;           // 書き込みコミットを実施するかのフラグ
  for (int i = 0; i < EEPROM_SIZE; i++) // データを書き込む時はbyte型
  {
    if (i >= EEPROM.length()) // EEPROMのサイズを超えないようチェック
    {
      Serial.println("Error: EEPROM address out of range.");
      return false;
    }
    old_value_tmp = EEPROM.read(i);
    // 書き込みデータがEEPROM内のデータと違う場合のみ書き込みをセット
    if (old_value_tmp != a_write_data.bval[i]) {
      EEPROM.write(i, a_write_data.bval[i]);
      flg_renew_tmp = true;
    }
  }

  if (true) {
    for (int i = 0; i < 15; i++) { // データを書き込む時はbyte型
      a_serial.print("L-idx:");
      a_serial.print(mrd_pddstr(i, 2, 0, false));
      a_serial.print(", id:");
      a_serial.print(mrd_pddstr(mrd_slice_bits(a_write_data.usaval[1][20 + i * 2], 1, 7), 2, 0, false)); // 2bit-7bit目:サーボID
      a_serial.print(", mt:");
      a_serial.print(mrd_pddstr(mrd_slice_bits(a_write_data.usaval[1][20 + i * 2], 0, 1), 1, 0, false)); // 1bit目:マウント有無
      a_serial.print(", cw:");
      a_serial.print(mrd_pddstr(mrd_slice_bits(a_write_data.usaval[1][20 + i * 2], 8, 1), 1, 0, false)); // 9bit目:正転逆転
      a_serial.print(", trm:");
      a_serial.print(mrd_pddstr(float(a_write_data.saval[1][21 + i * 2] / 100), 7, 2, true));
      a_serial.print("  R-idx: ");
      a_serial.print(mrd_pddstr(i, 2, 0, false));
      a_serial.print(", id:");
      a_serial.print(mrd_pddstr(mrd_slice_bits(a_write_data.usaval[1][50 + i * 2], 1, 7), 2, 0, false)); // 2bit-7bit目:サーボID
      a_serial.print(", mt:");
      a_serial.print(mrd_pddstr(mrd_slice_bits(a_write_data.usaval[1][50 + i * 2], 0, 1), 1, 0, false)); // 1bit目:マウント有無
      a_serial.print(", cw:");
      a_serial.print(mrd_pddstr(mrd_slice_bits(a_write_data.usaval[1][50 + i * 2], 8, 1), 1, 0, false)); // 9bit目:正転逆転
      a_serial.print(", trm:");
      a_serial.println(mrd_pddstr(float(a_write_data.saval[1][51 + i * 2] / 100), 7, 2, true));
    }
  }

  if (flg_renew_tmp) // 変更箇所があれば書き込みを実施
  {
    EEPROM.commit(); // 書き込みを確定する
    Serial.println("Value updated.");
    return true;
  } else {
    Serial.println("Value not changed.(same value)");
  }
  return false;
}

/// @brief EEPROMに設定値を書き込み, その後で読み込んで内容を確認し, シリアルポートに出力する.
/// @param a_write_data EEPROM書き込み用の配列データ.
/// @param a_do EEPROMの読み書きチェックを実施するかのブール値.
/// @param a_protect EEPROMの書き込み許可があるかどうかのブール値.
/// @param a_bhd ダンプリストの表示形式.(0:Bin, 1:Hex, 2:Dec)
/// @return EEPROMの書き込みと読み込みが成功した場合はtrueを, それ以外はfalseを返す.
bool mrd_eeprom_write_read_check(UnionEEPROM a_write_data, bool a_do, bool a_protect, int a_bhd) {
  if (!a_do) // EEPROMの読み書きチェックを実施するか
  {
    return false;
  }

  // EEPROM書き込みを実行
  Serial.println("Try to write EEPROM: ");
  mrd_eeprom_dump_to_serial(a_write_data, a_bhd, Serial); // 書き込み内容をダンプ表示

  if (mrd_eeprom_write(a_write_data, a_protect, Serial)) {
    Serial.println("...Write OK.");
  } else {
    Serial.println("...Write failed.");
    return false;
  }

  // EEPROM読み込みを実行
  Serial.println("Read EEPROM: ");
  UnionEEPROM read_data_tmp = mrd_eeprom_read();
  mrd_eeprom_dump_to_serial(read_data_tmp, a_bhd, Serial); // 読み込み内容をダンプ表示
  Serial.println("...Read completed.");

  return true;
}

//------------------------------------------------------------------------------------
//  各種オペレーション
//------------------------------------------------------------------------------------

/// @brief EEPROMから任意のshort型データを読み込む.
/// @param index_y 配列の一次元目(0~2).
/// @param index_x 配列の二次元目(0~89).
/// @return short型データを返す.
short mrd_eeprom_read_short(int index_y, int index_x) {
  return short(EEPROM.read(index_y * 90 + index_x));
}

/// @brief EEPROMから任意のbyte型データを読み込む.
/// @param index_y 配列の一次元目(0~2).
/// @param index_x 配列の二次元目(0~179).
/// @param low_high 下位ビットか上位ビットか. (0:low_bit, 1:high_bit)
/// @return byte型データを返す.
int8_t mrd_eeprom_read_byte(int index_y, int index_x, int low_high) //
{
  return int8_t(EEPROM.read(index_y * 180 + index_x * 2 + low_high));
}

#endif // __MERIDIAN_EEPROM_H__
