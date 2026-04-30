// mrd_eeprom.cpp
// EEPROM関連の関数実装

// ヘッダファイルの読み込み
#include "mrd_eeprom.h"
#include "mrd_util.h"

// ライブラリ導入
#include <EEPROM.h>

// EEPROM読み書き用共用体
UnionEEPROM eeprom_write_data; // EEPROM書き込み用
UnionEEPROM eeprom_read_data;  // EEPROM読み込み用

//==================================================================================================
//  EEPROM関数
//==================================================================================================

/// @brief EEPROMを初期化する
/// @param a_eeprom_size EEPROMのバイト長
/// @return 成功時はtrue, 失敗時はfalse
bool mrd_eeprom_init(int a_eeprom_size) {
  if (EEPROM.begin(a_eeprom_size)) {
    return true;
  } else {
  }
  return false;
}

/// @brief サーボ設定構造体からEEPROM格納用の配列データを作成する
/// @param a_sv サーボ設定を保持する構造体
/// @param a_mrd Meridianクラスのインスタンス
/// @return EEPROM格納用の配列データ (UnionEEPROM型)
UnionEEPROM mrd_eeprom_make_data_from_config(const ServoParam &a_sv, MERIDIANFLOW::Meridian &a_mrd) {
  UnionEEPROM array_tmp = {0};

  for (int i = 0; i < 15; i++) {
    // 各サーボのマウント状態と方向 (正転/逆転)
    uint16_t l_tmp = 0;
    uint16_t r_tmp = 0;

    // bit0 : マウント状態
    if (a_sv.ixl_mount[i])
      l_tmp |= 0x0001;
    if (a_sv.ixr_mount[i])
      r_tmp |= 0x0001;

    // bit1-7 : サーボID
    l_tmp |= static_cast<uint16_t>(a_sv.ixl_id[i] & 0x7F) << 1;
    r_tmp |= static_cast<uint16_t>(a_sv.ixr_id[i] & 0x7F) << 1;

    // bit8 : サーボ回転方向
    if (a_sv.ixl_cw[i] > 0)
      l_tmp |= 0x0100;
    if (a_sv.ixr_cw[i] > 0)
      r_tmp |= 0x0100;

    // マウント状態, ID, 回転方向データを格納
    array_tmp.saval[1][20 + i * 2] = l_tmp;
    array_tmp.saval[1][50 + i * 2] = r_tmp;

    // 各サーボのデフォルト直立角度を格納 (degree -> float short*100)
    array_tmp.saval[1][21 + i * 2] = a_mrd.float2HfShort(a_sv.ixl_trim[i]);
    array_tmp.saval[1][51 + i * 2] = a_mrd.float2HfShort(a_sv.ixr_trim[i]);
  }
  return array_tmp;
}

/// @brief EEPROMの内容を読み取って返す
/// @return UnionEEPROM形式の配列
UnionEEPROM mrd_eeprom_read() {
  UnionEEPROM read_data_tmp = {0};        // 明示的にゼロ初期化
  for (int i = 0; i < EEPROM_SIZE; i++) { // byte型でデータを読み込み
    read_data_tmp.bval[i] = EEPROM.read(i);
  }
  return read_data_tmp;
}

/// @brief EEPROMの内容を読み取りサーボ値構造体に反映する
/// @param a_sv サーボ設定を保持する構造体
/// @param a_monitor シリアルモニタにデータを表示するか
/// @param a_serial 出力シリアル
/// @return 完了時にtrueを返す
bool mrd_eeprom_load_servosettings(ServoParam &a_sv, bool a_monitor, HardwareSerial &a_serial) {
  a_serial.println("Load and set servo settings from EEPROM.");
  UnionEEPROM array_tmp = mrd_eeprom_read();
  for (int i = 0; i < a_sv.num_max; i++) {
    // 各サーボのマウント状態
    a_sv.ixl_mount[i] = static_cast<bool>(array_tmp.saval[1][20 + i * 2] & 0x0001); // bit0:マウント状態
    a_sv.ixr_mount[i] = static_cast<bool>(array_tmp.saval[1][50 + i * 2] & 0x0001); // bit0:マウント状態
    // 各サーボインデックスに対する実際のサーボ呼び出しID
    a_sv.ixl_id[i] = static_cast<uint8_t>(array_tmp.saval[1][20 + i * 2] >> 1 & 0x007F); // bit1-7:サーボID
    a_sv.ixr_id[i] = static_cast<uint8_t>(array_tmp.saval[1][50 + i * 2] >> 1 & 0x007F); // bit1-7:サーボID
    // 各サーボの回転方向 (正転/逆転)
    a_sv.ixl_cw[i] = static_cast<int8_t>((array_tmp.saval[1][20 + i * 2] >> 8) & 0x0001) ? 1 : -1; // bit8:回転方向
    a_sv.ixr_cw[i] = static_cast<int8_t>((array_tmp.saval[1][50 + i * 2] >> 8) & 0x0001) ? 1 : -1; // bit8:回転方向
    // 各サーボの直立デフォルト角度とトリム値 (*100格納値から小数点2桁で展開)
    a_sv.ixl_trim[i] = array_tmp.saval[1][21 + i * 2] / 100.0f;
    a_sv.ixr_trim[i] = array_tmp.saval[1][51 + i * 2] / 100.0f;

    if (a_monitor) {
      a_serial.print("L-idx:");
      a_serial.print(mrd_pddstr(i, 2, 0, false));
      a_serial.print(", id:");
      a_serial.print(mrd_pddstr(a_sv.ixl_id[i], 2, 0, false));
      a_serial.print(", mt:");
      a_serial.print(mrd_pddstr(a_sv.ixl_mount[i], 1, 0, false));
      a_serial.print(", cw:");
      a_serial.print(mrd_pddstr(a_sv.ixl_cw[i], 1, 0, true));
      a_serial.print(", trm:");
      a_serial.print(mrd_pddstr(a_sv.ixl_trim[i], 7, 2, true));
      a_serial.print("  R-idx: ");
      a_serial.print(mrd_pddstr(i, 2, 0, false));
      a_serial.print(", id:");
      a_serial.print(mrd_pddstr(a_sv.ixr_id[i], 2, 0, false));
      a_serial.print(", mt:");
      a_serial.print(mrd_pddstr(a_sv.ixr_mount[i], 1, 0, false));
      a_serial.print(", cw:");
      a_serial.print(mrd_pddstr(a_sv.ixr_cw[i], 1, 0, true));
      a_serial.print(", trm:");
      a_serial.println(mrd_pddstr(a_sv.ixr_trim[i], 7, 2, true));
    }
  }
  return true;
}

/// @brief EEPROM格納用配列データをシリアルにダンプ出力する
/// @param a_data EEPROM用の配列データ
/// @param a_bhd ダンプリストの表示形式 (0:Bin, 1:Hex, 2:Dec)
/// @return 完了時にtrueを返す
bool mrd_eeprom_dump_to_serial(UnionEEPROM a_data, int a_bhd, HardwareSerial &a_serial) {
  int len_tmp = EEPROM.length(); // EEPROMの長さ
  a_serial.print("EEPROM Length ");
  a_serial.print(len_tmp); // EEPROMの長さを表示
  a_serial.println("byte, 16bit Dump;");
  for (int i = 0; i < 270; i++) { // 読み取るデータはshort型で作成
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

/// @brief EEPROM格納用配列データをシリアルにダンプ出力する (起動時用)
/// @param a_do_dump 実行するかどうか
/// @param a_bhd ダンプリストの表示形式 (0:Bin, 1:Hex, 2:Dec)
/// @return 完了時にtrueを返す
bool mrd_eeprom_dump_at_boot(bool a_do_dump, int a_bhd, HardwareSerial &a_serial) {
  if (a_do_dump) {
    mrd_eeprom_dump_to_serial(mrd_eeprom_read(), a_bhd, a_serial);
    return true;
  }
  return false;
}

/// @brief EEPROM格納用配列データをEEPROMに書き込む
/// @param a_write_data EEPROM書き込み用の配列データ
/// @param a_flg_protect EEPROM書き込み保護フラグ (trueで書き込み禁止)
/// @param a_serial 出力シリアル
/// @param a_flg フラグの構造体 (参照渡し)
/// @return 書き込みが成功した場合はtrue, 書き込まなかった場合はfalse
bool mrd_eeprom_write(UnionEEPROM a_write_data, bool a_flg_protect, HardwareSerial &a_serial, MrdFlags &a_flg) {
  if (a_flg_protect) { // EEPROM書き込みフラグを確認
    return false;
  }
  if (a_flg.eeprom_protect) { // config.hのEEPROM書き込み保護を確認
    Serial.println("EEPROM is protected. To unprotect, please set 'EEPROM_PROTECT' to false.");
    return false;
  }

  // EEPROM書き込み
  byte old_value_tmp;                     // 既にEEPROMに書き込まれているデータ
  bool flg_renew_tmp = false;             // 書き込みコミット用フラグ
  for (int i = 0; i < EEPROM_SIZE; i++) { // byte型でデータを書き込み
    if (i >= EEPROM.length()) {           // EEPROMサイズを超えないか確認
      Serial.println("Error: EEPROM address out of range.");
      return false;
    }
    old_value_tmp = EEPROM.read(i);
    // EEPROMの内容と異なる場合のみ書き込みをセット
    if (old_value_tmp != a_write_data.bval[i]) {
      EEPROM.write(i, a_write_data.bval[i]);
      flg_renew_tmp = true;
    }
  }

  // 書き込み内容を表示
  for (int i = 0; i < 15; i++) {
    a_serial.print("L-idx:");
    a_serial.print(mrd_pddstr(i, 2, 0, false));
    a_serial.print(", id:");
    a_serial.print(mrd_pddstr(mrd_slice_bits(a_write_data.usaval[1][20 + i * 2], 1, 7), 2, 0, false)); // bit2-7:サーボID
    a_serial.print(", mt:");
    a_serial.print(mrd_pddstr(mrd_slice_bits(a_write_data.usaval[1][20 + i * 2], 0, 1), 1, 0, false)); // bit1:マウント状態
    a_serial.print(", cw:");
    a_serial.print(mrd_pddstr(mrd_slice_bits(a_write_data.usaval[1][20 + i * 2], 8, 1), 1, 0, false)); // bit9:正転/逆転
    a_serial.print(", trm:");
    a_serial.print(mrd_pddstr(float(a_write_data.saval[1][21 + i * 2] / 100), 7, 2, true));
    a_serial.print("  R-idx: ");
    a_serial.print(mrd_pddstr(i, 2, 0, false));
    a_serial.print(", id:");
    a_serial.print(mrd_pddstr(mrd_slice_bits(a_write_data.usaval[1][50 + i * 2], 1, 7), 2, 0, false)); // bit2-7:サーボID
    a_serial.print(", mt:");
    a_serial.print(mrd_pddstr(mrd_slice_bits(a_write_data.usaval[1][50 + i * 2], 0, 1), 1, 0, false)); // bit1:マウント状態
    a_serial.print(", cw:");
    a_serial.print(mrd_pddstr(mrd_slice_bits(a_write_data.usaval[1][50 + i * 2], 8, 1), 1, 0, false)); // bit9:正転/逆転
    a_serial.print(", trm:");
    a_serial.println(mrd_pddstr(float(a_write_data.saval[1][51 + i * 2] / 100), 7, 2, true));
  }

  if (flg_renew_tmp) { // 変更がある場合は書き込みを実行
    EEPROM.commit();   // 書き込みを確定
    Serial.println("Value updated.");
    return true;
  } else {
    Serial.println("Value not changed.(same value)");
  }
  return false;
}

/// @brief EEPROMに設定を書き込み, 読み取って内容を検証し, シリアルポートに出力する
/// @param a_write_data EEPROM書き込み用の配列データ
/// @param a_do EEPROM読み書きチェックのブール値
/// @param a_protect EEPROM書き込み保護フラグ (trueで書き込み禁止)
/// @param a_bhd ダンプリストの表示形式 (0:Bin, 1:Hex, 2:Dec)
/// @param a_flg フラグの構造体 (参照渡し)
/// @return 書き込みと読み込みが成功した場合はtrue, それ以外はfalse
bool mrd_eeprom_write_read_check(UnionEEPROM a_write_data, bool a_do, bool a_protect, int a_bhd, MrdFlags &a_flg) {
  if (!a_do) { // EEPROM読み書きチェックを実行するかどうか
    return false;
  }

  // EEPROM書き込みを実行
  Serial.println("Try to write EEPROM: ");
  mrd_eeprom_dump_to_serial(a_write_data, a_bhd, Serial); // 書き込み内容をダンプ

  if (mrd_eeprom_write(a_write_data, a_protect, Serial, a_flg)) {
    Serial.println("...Write OK.");
  } else {
    Serial.println("...Write failed.");
    return false;
  }

  // EEPROM読み込みを実行
  Serial.println("Read EEPROM: ");
  UnionEEPROM read_data_tmp = mrd_eeprom_read();
  mrd_eeprom_dump_to_serial(read_data_tmp, a_bhd, Serial); // 読み込み内容をダンプ
  Serial.println("...Read completed.");

  return true;
}

//------------------------------------------------------------------------------------
//  各種オペレーション
//------------------------------------------------------------------------------------

/// @brief EEPROMから1バイトを読み取りshort型にキャストして返す
/// @param index_y 配列の第1次元 (0~2)
/// @param index_x 配列の第2次元 (0~89)
/// @return 1バイトをshort型にキャストした値
short mrd_eeprom_read_short(int index_y, int index_x) {
  return short(EEPROM.read(index_y * 90 + index_x));
}

/// @brief EEPROMから任意のbyte型データを読み取る
/// @param index_y 配列の第1次元 (0~2)
/// @param index_x 配列の第2次元 (0~179)
/// @param low_high 下位または上位ビット (0:low_bit, 1:high_bit)
/// @return byte型データ
int8_t mrd_eeprom_read_byte(int index_y, int index_x, int low_high) {
  return int8_t(EEPROM.read(index_y * 180 + index_x * 2 + low_high));
}
