#ifndef __MERIDIAN_WIFI_H__
#define __MERIDIAN_WIFI_H__

// ヘッダファイルの読み込み

// ライブラリ導入
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

//==================================================================================================
//  WiFi関数宣言
//==================================================================================================

/// @brief WiFiを初期化する
/// @param a_ssid WiFiアクセスポイントのSSID
/// @param a_pass WiFiアクセスポイントのパスワード
/// @param a_serial 出力シリアル
/// @return 成功時はtrue, 失敗時はfalse
bool mrd_wifi_init(const char *a_ssid, const char *a_pass, HardwareSerial &a_serial);

/// @brief UDP経由でデータを受信しMeridim配列に格納する
/// @param a_meridim_bval バイト型のMeridim配列
/// @param a_len バイト型Meridim配列の長さ
/// @return 受信した場合はtrue, 受信しなかった場合はfalse
bool mrd_wifi_udp_receive(byte *a_meridim_bval, int a_len);

/// @brief Meridim配列データをUDP経由でWIFI_SEND_IP, UDP_SEND_PORTへ送信する
/// @param a_meridim_bval バイト型のMeridim配列
/// @param a_len バイト型Meridim配列の長さ
/// @return 成功時はtrue, 失敗時はfalse
/// 内部でWIFI_SEND_IP, UDP_SEND_PORTを使用
bool mrd_wifi_udp_send(byte *a_meridim_bval, int a_len);

#endif // __MERIDIAN_WIFI_H__
