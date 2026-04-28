// mrd_wifi.cpp
// WiFi関連の関数実装

#include "mrd_wifi.h"

//==================================================================================================
//  WiFi関数
//==================================================================================================

/// @brief WiFiを初期化する
/// @param a_udp WiFiUDPインスタンス
/// @param a_ssid WiFiアクセスポイントのSSID
/// @param a_pass WiFiアクセスポイントのパスワード
/// @param a_serial 出力シリアル
/// @return 成功時はtrue, 失敗時はfalse
bool mrd_wifi_init(WiFiUDP &a_udp, const char *a_ssid, const char *a_pass,
                   HardwareSerial &a_serial) {
  WiFi.disconnect(true, true); // 新規接続のためWiFi接続をリセット
  delay(100);
  WiFi.begin(a_ssid, a_pass); // WiFiに接続
  int i = 0;
  while (WiFi.status() !=
         WL_CONNECTED) { // https://www.arduino.cc/en/Reference/WiFiStatus 戻り値一覧
    i++;
    if (i % 10 == 0) { // 0.5秒ごとに接続状態を出力
      a_serial.print(".");
    }
    delay(50);     // 接続完了までループ
    if (i > 200) { // 10秒後タイムアウト
      a_serial.println("Wifi init TIMEOUT.");
      return false;
    }
  }
  a_udp.begin(UDP_RECV_PORT);
  return true;
}

/// @brief UDP経由でデータを受信しMeridim配列に格納する
/// @param a_meridim_bval バイト型のMeridim配列
/// @param a_len バイト型Meridim配列の長さ
/// @param a_udp 使用するWiFiUDPインスタンス
/// @return 受信した場合はtrue, 受信しなかった場合はfalse
bool mrd_wifi_udp_receive(byte *a_meridim_bval, int a_len, WiFiUDP &a_udp) {
  if (a_udp.parsePacket() >= a_len) { // 受信バッファにデータがあるか確認
    a_udp.read(a_meridim_bval, a_len); // データを受信
    return true;
  }
  return false; // バッファにデータなし
}

/// @brief Meridim配列データをUDP経由でWIFI_SEND_IP, UDP_SEND_PORTへ送信する
/// @param a_meridim_bval バイト型のMeridim配列
/// @param a_len バイト型Meridim配列の長さ
/// @param a_udp 使用するWiFiUDPインスタンス
/// @return 成功時はtrue, 失敗時はfalse
/// 内部でWIFI_SEND_IP, UDP_SEND_PORTを使用
bool mrd_wifi_udp_send(byte *a_meridim_bval, int a_len, WiFiUDP &a_udp) {
  int result = a_udp.beginPacket(WIFI_SEND_IP, UDP_SEND_PORT); // UDPパケット開始
  if (result == 0) {
    return false; // パケット開始失敗
  }

  size_t bytes_written = a_udp.write(a_meridim_bval, a_len); // データ書き込み
  if (bytes_written != a_len) {
    return false; // 書き込みサイズ不一致
  }

  result = a_udp.endPacket(); // UDPパケット終了
  return (result == 1);       // 成功時は1を返す
}
