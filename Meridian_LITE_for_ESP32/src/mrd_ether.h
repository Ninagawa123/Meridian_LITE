#ifndef __MERIDIAN_ETHER_H__
#define __MERIDIAN_ETHER_H__

// ヘッダファイルの読み込み

// ライブラリ導入 (標準Ethernetライブラリ)
#include <Ethernet.h>

//==================================================================================================
// Ethernet関数宣言
//==================================================================================================

// MACアドレス文字列をバイト配列にパースする関数（改良版）
bool parseMacAddress(const char *macStr, byte *macBytes);

/// @brief Ethernetを初期化する (文字列IP設定版)
/// @param a_cs_pin W5500のCSピン番号
/// @param mac_address MACアドレスバイト配列
/// @param a_serial 出力シリアル
/// @return 成功時はtrue, 失敗時はfalse
bool mrd_ether_init(int a_cs_pin, byte *mac_address, HardwareSerial &a_serial);

/// @brief UDP経由でデータを受信しMeridim配列に格納する
/// @param a_meridim_bval バイト型のMeridim配列
/// @param a_len バイト型Meridim配列の長さ
/// @return 受信した場合はtrue, 受信しなかった場合はfalse
bool mrd_ether_udp_receive(byte *a_meridim_bval, int a_len);

/// @brief Meridim配列データをUDP経由で指定IPとポートに送信する
/// @param a_meridim_bval バイト型のMeridim配列
/// @param a_len バイト型Meridim配列の長さ
/// @param a_send_ip 送信先IPアドレス
/// @return 完了時にtrueを返す
bool mrd_ether_udp_send(byte *a_meridim_bval, int a_len, IPAddress a_send_ip);

#endif // __MERIDIAN_ETHER_H__
