#ifndef __MERIDIAN_EEPROM_H__
#define __MERIDIAN_EEPROM_H__

// ヘッダファイルの読み込み
#include "config.h"
#include "mrd_common.h"
#include "mrd_util.h"

// ライブラリ導入
#include <EEPROM.h>
#include <Meridian.h>

// グローバル変数のextern宣言 (実体はmain.cppで定義)
extern MERIDIANFLOW::Meridian mrd;
extern MrdFlags flg;

// UnionEEPROM は mrd_common.h で定義

//==================================================================================================
//  EEPROM関数宣言
//==================================================================================================

/// @brief EEPROMを初期化する
/// @param a_eeprom_size EEPROMのバイト長
/// @return 成功時はtrue, 失敗時はfalse
bool mrd_eeprom_init(int a_eeprom_size);

/// @brief サーボ設定構造体からEEPROM格納用の配列データを作成する
/// @param a_sv サーボ設定を保持する構造体
/// @return EEPROM格納用の配列データ (UnionEEPROM型)
UnionEEPROM mrd_eeprom_make_data_from_config(const ServoParam &a_sv);

/// @brief EEPROMの内容を読み取って返す
/// @return UnionEEPROM形式の配列
UnionEEPROM mrd_eeprom_read();

/// @brief EEPROMの内容を読み取りサーボ値構造体に反映する
/// @param a_sv サーボ設定を保持する構造体
/// @param a_monitor シリアルモニタにデータを表示するか
/// @param a_serial 出力シリアル
/// @return 完了時にtrueを返す
bool mrd_eeprom_load_servosettings(ServoParam &a_sv, bool a_monitor, HardwareSerial &a_serial);

/// @brief EEPROM格納用配列データをシリアルにダンプ出力する
/// @param a_data EEPROM用の配列データ
/// @param a_bhd ダンプリストの表示形式 (0:Bin, 1:Hex, 2:Dec)
/// @param a_serial 出力シリアル
/// @return 完了時にtrueを返す
bool mrd_eeprom_dump_to_serial(UnionEEPROM a_data, int a_bhd, HardwareSerial &a_serial);

/// @brief EEPROM格納用配列データをシリアルにダンプ出力する (起動時用)
/// @param a_do_dump 実行するかどうか
/// @param a_bhd ダンプリストの表示形式 (0:Bin, 1:Hex, 2:Dec)
/// @param a_serial 出力シリアル
/// @return 完了時にtrueを返す
bool mrd_eeprom_dump_at_boot(bool a_do_dump, int a_bhd, HardwareSerial &a_serial);

/// @brief EEPROM格納用配列データをEEPROMに書き込む
/// @param a_write_data EEPROM書き込み用の配列データ
/// @param a_flg_protect EEPROM書き込み保護フラグ (trueで書き込み禁止)
/// @param a_serial 出力シリアル
/// @return 書き込みが成功した場合はtrue, 書き込まなかった場合はfalse
bool mrd_eeprom_write(UnionEEPROM a_write_data, bool a_flg_protect, HardwareSerial &a_serial);

/// @brief EEPROMに設定を書き込み, 読み取って内容を検証し, シリアルポートに出力する
/// @param a_write_data EEPROM書き込み用の配列データ
/// @param a_do EEPROM読み書きチェックのブール値
/// @param a_protect EEPROM書き込み保護フラグ (trueで書き込み禁止)
/// @param a_bhd ダンプリストの表示形式 (0:Bin, 1:Hex, 2:Dec)
/// @return 書き込みと読み込みが成功した場合はtrue, それ以外はfalse
bool mrd_eeprom_write_read_check(UnionEEPROM a_write_data, bool a_do, bool a_protect, int a_bhd);

//------------------------------------------------------------------------------------
//  各種オペレーション
//------------------------------------------------------------------------------------

/// @brief EEPROMから1バイトを読み取りshort型にキャストして返す
/// @param index_y 配列の第1次元 (0~2)
/// @param index_x 配列の第2次元 (0~89)
/// @return 1バイトをshort型にキャストした値
short mrd_eeprom_read_short(int index_y, int index_x);

/// @brief EEPROMから任意のbyte型データを読み取る
/// @param index_y 配列の第1次元 (0~2)
/// @param index_x 配列の第2次元 (0~179)
/// @param low_high 下位または上位ビット (0:low_bit, 1:high_bit)
/// @return byte型データ
int8_t mrd_eeprom_read_byte(int index_y, int index_x, int low_high);

#endif // __MERIDIAN_EEPROM_H__
