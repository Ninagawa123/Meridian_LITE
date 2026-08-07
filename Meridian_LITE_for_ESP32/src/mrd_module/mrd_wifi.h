#ifndef __MERIDIAN_WIFI_H__
#define __MERIDIAN_WIFI_H__

// ヘッダファイルの読み込み
#include "../config.h"
#include "../keys.h"

// ライブラリ導入
#include <WiFi.h>
#include <WiFiUdp.h>

// Global variable declaration
extern WiFiUDP udp; // wifi setting

//==================================================================================================
//  関数プロトタイプ宣言
//==================================================================================================

/// @brief 送信先IPアドレスとポートをランタイムで設定する.
/// @param a_ip        送信先IPアドレス文字列. nullptrの場合は変更しない.
/// @param a_port      送信先ポート番号. 0の場合は変更しない.
/// @param a_recv_port 受信ポート番号. 0の場合は変更しない.
void mrd_wifi_set_dest(const char *a_ip, uint16_t a_port = 0, uint16_t a_recv_port = 0);

/// @brief wifiを初期化する.
/// @param a_ssid     WifiアクセスポイントのSSID.
/// @param a_pass     Wifiアクセスポイントのパスワード.
/// @param a_serial   出力先シリアルの指定.
/// @param a_fixed_ip ESP32の固定IPアドレス. nullptrの場合はkeys.hのデフォルトを使用.
/// @param a_gateway  ゲートウェイIPアドレス. nullptrの場合はkeys.hのデフォルトを使用.
/// @param a_subnet   サブネットマスク. nullptrの場合はkeys.hのデフォルトを使用.
/// @return 初期化に成功した場合はtrueを, 失敗した場合はfalseを返す.
bool mrd_wifi_init(WiFiUDP &a_udp, const char *a_ssid, const char *a_pass,
                   HardwareSerial &a_serial,
                   const char *a_fixed_ip = nullptr,
                   const char *a_gateway  = nullptr,
                   const char *a_subnet   = nullptr);

/// @brief 第一引数のMeridim配列にUDP経由でデータを受信, 格納する.
/// @param a_meridim_bval バイト型のMeridim配列
/// @param a_len バイト型のMeridim配列の長さ
/// @param a_udp 使用するWiFiUDPのインスタンス
/// @return 受信した場合はtrueを, 受信しなかった場合はfalseを返す.
bool mrd_wifi_udp_receive(byte *a_meridim_bval, int a_len, WiFiUDP &a_udp);

/// @brief 第一引数のMeridim配列のデータをUDP経由で送信先に送信する.
/// @param a_meridim_bval バイト型のMeridim配列
/// @param a_len バイト型のMeridim配列の長さ
/// @param a_udp 使用するWiFiUDPのインスタンス
/// @return 送信完了時にtrueを返す.
bool mrd_wifi_udp_send(byte *a_meridim_bval, int a_len, WiFiUDP &a_udp);

#endif // __MERIDIAN_WIFI_H__
