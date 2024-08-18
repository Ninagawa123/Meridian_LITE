#ifndef __MERIDIAN_EEPROM_H__
#define __MERIDIAN_EEPROM_H__

// ヘッダファイルの読み込み
#include "config.h"
#include "main.h"

// ライブラリ導入
#include <EEPROM.h>

// EEPROM読み書き用共用体
typedef union {
  uint8_t bval[EEPROM_SIZE];            // 1バイト単位で540個のデータを持つ
  short saval[3][int(EEPROM_SIZE / 4)]; // short型で3*90個の配列データを持つ
  short sval[int(EEPROM_SIZE / 2)];     // short型で270個のデータを持つ
} UnionEEPROM;
UnionEEPROM eeprom_write_data; // EEPROM書き込み用
UnionEEPROM eeprom_read_data;  // EEPROM読み込み用

//================================================================================================================
//  EEPROM関連の処理
//================================================================================================================

/// @brief EEPROMの初期化
/// @param a_eeprom_size EEPROMのバイト長
/// @return 初期化が成功すればtrue, 失敗ならfalseを返す.
bool mrd_eeprom_init(int a_eeprom_size) {
  Serial.print("Initializing EEPROM... ");
  if (EEPROM.begin(a_eeprom_size)) {
    Serial.println("OK.");
    return true;
  } else {
    Serial.println("Failed.");
  }
  return false;
}

/// @brief config.hにあるサーボの諸設定からEEPROM格納用の配列データを作成する.
/// @return config.hから作成したEEPROM格納用の配列データを返す.
UnionEEPROM mrd_eeprom_make_data_from_config() {
  UnionEEPROM array_tmp = {0};
  for (int i = 0; i < 15; i++) {
    // 各サーボのマウントありなし（0:サーボなし, +:サーボあり順転, -:サーボあり逆転）
    // 例: IXL_MT[20] = -21; → FUTABA_RSxTTLサーボを逆転設定でマウント
    array_tmp.saval[0][20 + i * 2] = short(sv.ixl_mount[i] * sv.ixl_cw[i]);
    array_tmp.saval[0][50 + i * 2] = short(sv.ixr_mount[i] * sv.ixr_cw[i]);
    // 各サーボの直立デフォルト値 degree
    array_tmp.saval[1][21 + i * 2] = mrd.float2HfShort(sv.ixl_trim[i]);
    array_tmp.saval[1][51 + i * 2] = mrd.float2HfShort(sv.ixr_trim[i]);
  };
  return array_tmp;
}

/// @brief EEPROMの内容を読み込んで返す.
/// @return UnionEEPROM のフォーマットで配列を返す.
UnionEEPROM mrd_eeprom_read() {
  UnionEEPROM read_data_tmp;
  for (int i = 0; i < EEPROM_SIZE; i++) // データを読み込む時はbyte型
  {
    read_data_tmp.bval[i] = EEPROM.read(i);
  }
  return read_data_tmp;
}

/// @brief EEPROM格納用の配列データをシリアルにダンプ出力する.
/// @param a_data EEPROM用の配列データ.
/// @param a_bhd ダンプリストの表示形式.(0:Bin, 1:Hex, 2:Dec)
/// @return 終了時にtrueを返す.
bool mrd_eeprom_dump_to_serial(UnionEEPROM a_data, int a_bhd) {
  int len_tmp = EEPROM.length(); // EEPROMの長さ
  Serial.print("EEPROM Length ");
  Serial.print(len_tmp); // EEPROMの長さ表示
  Serial.println("byte, 16bit Dump;");
  for (int i = 0; i < 270; i++) // 読み込むデータはshort型で作成
  {
    if (a_bhd == 0) {
      Serial.print(a_data.sval[i], BIN);
    } else if (a_bhd == 1) {
      Serial.print(a_data.sval[i], HEX);
    } else {
      Serial.print(a_data.sval[i], DEC);
    }
    if ((i == 89) or (i == 179) or (i == 269)) {
      Serial.println();
    } else {
      Serial.print("/");
    }
  }
  return true;
}

/// @brief EEPROM格納用の配列データをシリアルにダンプ出力する.(起動時用)
/// @param a_do_dump 実施するか否か.
/// @param a_bhd ダンプリストの表示形式.(0:Bin, 1:Hex, 2:Dec)
/// @return 終了時にtrueを返す.
bool mrd_eeprom_dump_at_boot(bool a_do_dump, int a_bhd) {
  if (a_do_dump) {
    mrd_eeprom_dump_to_serial(mrd_eeprom_read(), a_bhd);
    return true;
  }
  return false;
}

/// @brief EEPROMにEEPROM格納用の配列データを書き込む.
/// @param a_write_data EEPROM書き込み用の配列データ.
/// @param a_flg_protect EEPROMの書き込み許可があるかどうかのブール値.
/// @return EEPROMの書き込みと読み込みが成功した場合はtrueを, 書き込まなかった場合はfalseを返す.
bool mrd_eeprom_write(UnionEEPROM a_write_data, bool a_flg_protect) {
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
  if (flg_renew_tmp) // 変更箇所があれば書き込みを実施
  {
    EEPROM.commit(); // 書き込みを確定する
    Serial.print("Value updated ");
    return true;
  }else{
    Serial.print("Same value ");
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
  mrd_eeprom_dump_to_serial(a_write_data, a_bhd); // 書き込み内容をダンプ表示

  if (mrd_eeprom_write(a_write_data, a_protect)) {
    Serial.println("...Write OK.");
  } else {
    Serial.println("...Write failed.");
    return false;
  }

  // EEPROM読み込みを実行
  Serial.println("Read EEPROM: ");
  UnionEEPROM read_data_tmp = mrd_eeprom_read();
  mrd_eeprom_dump_to_serial(read_data_tmp, a_bhd); // 読み込み内容をダンプ表示
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
