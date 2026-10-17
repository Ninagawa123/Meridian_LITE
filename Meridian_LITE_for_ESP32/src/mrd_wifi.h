#ifndef __MERIDIAN_WIFI_H__
#define __MERIDIAN_WIFI_H__

// ヘッダファイルの読み込み
#include "config.h"
#include "keys.h"
#include "main.h"

// ライブラリ導入
#include <WiFi.h>
#include <WiFiUdp.h>
WiFiUDP udp; // wifi設定

//==================================================================================================
//  Wifi 関連の処理
//==================================================================================================

/// @brief wifiに接続し, 省電力を切って, UDPの受信を開始する.
/// @param a_udp 接続後に受信を開始するWiFiUDPのインスタンス(cfg.udp_recv_port).
/// @param a_ssid WifiアクセスポイントのSSID.
/// @param a_pass Wifiアクセスポイントのパスワード.
/// @param a_serial 出力先シリアルの指定.
/// @return 初期化に成功した場合はtrueを, 失敗した場合はfalseを返す.
bool mrd_wifi_init(WiFiUDP &a_udp, const char *a_ssid, const char *a_pass,
                   HardwareSerial &a_serial) {
  WiFi.disconnect(true, true); // 新しい接続のためにWiFi接続をリセット
  delay(100);
  WiFi.begin(a_ssid, a_pass); // Wifiに接続
  int i = 0;
  while (WiFi.status() !=
         WL_CONNECTED) { // https://www.arduino.cc/en/Reference/WiFiStatus 戻り値一覧
    i++;
    if (i % 10 == 0) { // 0.5秒ごとに接続状況を出力
      a_serial.print(".");
    }
    delay(50);     // 接続が完了するまでループで待つ
    if (i > 200) { // 10秒でタイムアウト
      a_serial.println("Wifi init TIMEOUT.");
      return false;
    }
  }
  // 省電力(モデムスリープ)を切る. 既定のままだとAPからの受信がビーコン間隔まで遅れ,
  // UDPの受信待ちのタイムアウトとフレームのスキップが増える
  WiFi.setSleep(false);
  if (a_udp.begin(cfg.udp_recv_port) != 1) {
    a_serial.println("UDP begin failed (port " + String(cfg.udp_recv_port) + ").");
    return false;
  }
  return true;
}

/// @brief 第一引数のMeridim配列にUDP経由でデータを受信, 格納する.
/// @param a_meridim_bval バイト型のMeridim配列
/// @param a_len バイト型のMeridim配列の長さ
/// @param a_udp 使用するWiFiUDPのインスタンス
///        届いているパケットをすべて読み, 最新の1つを使う. 遅れて届いたパケットは捨てずに次のフレームで使うため,
///        WiFiの遅延の揺れを吸収でき, 受信待ちのタイムアウトが減る. 複数が溜まっていれば古いものは捨て,
///        数フレーム遅れの指令を使い続けないようにする(捨てた分は連番の飛びになる).
/// @return 受信した場合はtrueを, 受信しなかった場合はfalseを返す.
bool mrd_wifi_udp_receive(byte *a_meridim_bval, int a_len, WiFiUDP &a_udp) {
  bool rcvd_tmp = false;
  while (a_udp.parsePacket() > 0) { // データの受信バッファ確認
    if (a_udp.available() >= a_len) {
      a_udp.read(a_meridim_bval, a_len); // データの受信(後に届いたもので上書きする)
      rcvd_tmp = true;
    }
    a_udp.flush(); // 読み残し(長さの違うパケット等)を破棄する. 残っていると次のパケットを受け取れない
  }
  return rcvd_tmp;
}

/// @brief 第一引数のMeridim配列のデータをUDP経由で送り先(cfg.wifi_send_ip, cfg.udp_send_port)に送信する.
/// @param a_meridim_bval バイト型のMeridim配列
/// @param a_len バイト型のMeridim配列の長さ
/// @param a_udp 使用するWiFiUDPのインスタンス
/// @return 送信できた場合はtrueを返す(パケットの開始・書き込み・送信のどれかに失敗したらfalse).
/// ※cfg.wifi_send_ip, cfg.udp_send_portを関数内で使用.
bool mrd_wifi_udp_send(byte *a_meridim_bval, int a_len, WiFiUDP &a_udp) {
  // 送り先は初回に1度だけ変換する. 文字列のまま渡すと毎回名前解決(gethostbyname)を通り, 送信が遅くなる
  static IPAddress send_ip_tmp;
  static bool parsed_tmp = send_ip_tmp.fromString(cfg.wifi_send_ip);
  if (!parsed_tmp) {
    static bool warned_tmp = false; // 送れないことを最初の1回だけ表示する
    if (!warned_tmp) {
      warned_tmp = true;
      Serial.println("ERROR: WIFI_SEND_IP \"" + String(cfg.wifi_send_ip) +
                     "\" is not an IP address (a.b.c.d). Nothing is sent to the PC. Check keys.h or EEPROM P5.");
    }
    return false;
  }
  return a_udp.beginPacket(send_ip_tmp, cfg.udp_send_port) == 1 && // UDPパケットの開始
         a_udp.write(a_meridim_bval, a_len) == size_t(a_len) &&    // データの書き込み
         a_udp.endPacket() == 1;                                   // UDPパケットの終了(送信)
}

#endif // __MERIDIAN_WIFI_H__
