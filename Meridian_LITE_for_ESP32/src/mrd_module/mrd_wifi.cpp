// mrd_wifi.cpp - WiFi handling implementation

#include "mrd_wifi.h"

// Global variable
WiFiUDP udp; // wifi setting

// ランタイム送信先設定 (初期値はkeys.hのデフォルト値, EEPROM_LOADで上書き可)
static char     s_send_ip[16]  = WIFI_SEND_IP;
static uint16_t s_send_port    = UDP_SEND_PORT;
static uint16_t s_recv_port    = UDP_RECV_PORT;

//==================================================================================================
//  Wifi 関連の処理
//==================================================================================================

/// @brief 送信先IPアドレスとポートをランタイムで設定する.
/// @param a_ip   送信先IPアドレス文字列. nullptrの場合は変更しない.
/// @param a_port 送信先ポート番号. 0の場合は変更しない.
/// @param a_recv_port 受信ポート番号. 0の場合は変更しない.
void mrd_wifi_set_dest(const char *a_ip, uint16_t a_port, uint16_t a_recv_port) {
  if (a_ip && a_ip[0] != '\0') {
    strncpy(s_send_ip, a_ip, 15);
    s_send_ip[15] = '\0';
  }
  if (a_port > 0)      s_send_port = a_port;
  if (a_recv_port > 0) s_recv_port = a_recv_port;
}

/// @brief wifiを初期化する.
/// @param a_ssid      WifiアクセスポイントのSSID.
/// @param a_pass      Wifiアクセスポイントのパスワード.
/// @param a_serial    出力先シリアルの指定.
/// @param a_fixed_ip  ESP32の固定IPアドレス. nullptrの場合はkeys.hのデフォルトを使用.
/// @param a_gateway   ゲートウェイIPアドレス. nullptrの場合はkeys.hのデフォルトを使用.
/// @param a_subnet    サブネットマスク. nullptrの場合はkeys.hのデフォルトを使用.
/// @return 初期化に成功した場合はtrueを, 失敗した場合はfalseを返す.
bool mrd_wifi_init(WiFiUDP &a_udp, const char *a_ssid, const char *a_pass,
                   HardwareSerial &a_serial,
                   const char *a_fixed_ip, const char *a_gateway, const char *a_subnet) {
  WiFi.disconnect(true, true);
  delay(100);
  WiFi.setSleep(false); // must precede WiFi.begin() to apply WIFI_PS_NONE at STA_START
  WiFi.begin(a_ssid, a_pass);
  int i = 0;
  while (WiFi.status() != WL_CONNECTED) {
    if (++i > 200) { a_serial.println("Wifi init TIMEOUT."); return false; }
    if (i % 10 == 0) a_serial.print(".");
    delay(50);
  }
#if MODE_FIXED_IP
  {
    const char *use_ip  = (a_fixed_ip && a_fixed_ip[0]) ? a_fixed_ip  : FIXED_IP_ADDR;
    const char *use_gw  = (a_gateway  && a_gateway[0])  ? a_gateway   : FIXED_IP_GATEWAY;
    const char *use_sn  = (a_subnet   && a_subnet[0])   ? a_subnet    : FIXED_IP_SUBNET;
    IPAddress local_IP, gateway, subnet;
    local_IP.fromString(use_ip);
    if (WiFi.localIP() != local_IP) {
      gateway.fromString(use_gw);
      subnet.fromString(use_sn);
      WiFi.config(local_IP, gateway, subnet);
      delay(100);
      a_serial.println("Fixed IP set via WiFi.config.");
    }
  }
#endif
  a_udp.begin(s_recv_port);
  return true;
}

/// @brief 第一引数のMeridim配列にUDP経由でデータを受信, 格納する.
/// @param a_meridim_bval バイト型のMeridim配列
/// @param a_len バイト型のMeridim配列の長さ
/// @param a_udp 使用するWiFiUDPのインスタンス
/// @return 受信した場合はtrueを, 受信しなかった場合はfalseを返す.
bool mrd_wifi_udp_receive(byte *a_meridim_bval, int a_len, WiFiUDP &a_udp) {
  if (a_udp.parsePacket() >= a_len) // データの受信バッファ確認
  {
    a_udp.read(a_meridim_bval, a_len); // データの受信
    return true;
  }
  return false; // バッファにデータがない
}

/// @brief 第一引数のMeridim配列のデータをUDP経由で送信先に送信する.
/// @param a_meridim_bval バイト型のMeridim配列
/// @param a_len バイト型のMeridim配列の長さ
/// @param a_udp 使用するWiFiUDPのインスタンス
/// @return 送信完了時にtrueを返す.
bool mrd_wifi_udp_send(byte *a_meridim_bval, int a_len, WiFiUDP &a_udp) {
  a_udp.beginPacket(s_send_ip, s_send_port);
  a_udp.write(a_meridim_bval, a_len);
  return a_udp.endPacket(); // 0=fail, 1=success
}
