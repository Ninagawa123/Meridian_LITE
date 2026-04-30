// mrd_ether.cpp
// Ethernet関連の関数実装

// ヘッダファイルの読み込み
#include "mrd_ether.h"
#include "keys.h"
#include "mrd_util.h"

// ライブラリ導入 (標準Ethernetライブラリ)
#include <EthernetUdp.h>
#include <SPI.h>

EthernetUDP udp_et; // Ethernet設定

//==================================================================================================
// Ethernetヘルパー関数
//==================================================================================================

// 文字が有効な16進数かチェック
bool isValidHexChar(char c) {
  return (c >= '0' && c <= '9') ||
         (c >= 'A' && c <= 'F') ||
         (c >= 'a' && c <= 'f');
}

// 16進数文字をバイト値に変換
byte hexCharToByte(char c) {
  if (c >= '0' && c <= '9')
    return c - '0';
  if (c >= 'A' && c <= 'F')
    return c - 'A' + 10;
  if (c >= 'a' && c <= 'f')
    return c - 'a' + 10;
  return 0; // エラー (通常ここには到達しない)
}

// 文字列から余分なスペースを削除して正規化
String normalizeMacString(const char *macStr) {
  String normalized = "";
  for (int i = 0; macStr[i] != '\0'; i++) {
    if (macStr[i] != ' ' && macStr[i] != '\t') {
      normalized += macStr[i];
    }
  }
  return normalized;
}

// MACアドレス文字列をバイト配列にパース (改良版)
bool parseMacAddress(const char *macStr, byte *macBytes) {
  if (!macStr || !macBytes) {
    return false;
  }

  // 余分なスペースを削除して正規化
  String normalizedMac = normalizeMacString(macStr);
  const char *cleanMac = normalizedMac.c_str();

  // 正規化後の長さチェック (17文字必要: "XX:XX:XX:XX:XX:XX")
  if (normalizedMac.length() != 17) {
    return false;
  }

  // 各バイトをパース
  for (int i = 0; i < 6; i++) {
    // 16進数文字の位置を計算
    int pos1 = i * 3;
    int pos2 = i * 3 + 1;
    int colonPos = i * 3 + 2;

    // 16進数文字を検証
    if (!isValidHexChar(cleanMac[pos1]) || !isValidHexChar(cleanMac[pos2])) {
      return false;
    }

    // コロンをチェック (最後のバイト以外)
    if (i < 5 && cleanMac[colonPos] != ':') {
      return false;
    }

    // 16進数文字をバイトに変換
    byte highNibble = hexCharToByte(cleanMac[pos1]);
    byte lowNibble = hexCharToByte(cleanMac[pos2]);
    macBytes[i] = (highNibble << 4) | lowNibble;
  }

  return true;
}

/// @brief Ethernetを初期化する (文字列IP設定版)
/// @param a_cs_pin W5500のCSピン番号
/// @param mac_address MACアドレスバイト配列
/// @param a_serial 出力シリアル
/// @return 成功時はtrue, 失敗時はfalse
bool mrd_ether_init(int a_cs_pin, byte *mac_address, HardwareSerial &a_serial) {
  // MACアドレスを表示
  a_serial.print("Wired LAN MAC Address: ");
  for (int i = 0; i < 6; i++) {
    if (mac_address[i] < 16)
      a_serial.print("0");
    a_serial.print(mac_address[i], HEX);
    if (i < 5)
      a_serial.print(":");
  }
  a_serial.println();
  a_serial.print("Initializing Ethernet... ");

  // IPアドレス文字列をパース
  IPAddress local_ip = mrd_parse_ip_address(ETHER_LOCAL_IP, a_serial);
  IPAddress gateway = mrd_parse_ip_address(ETHER_GATEWAY, a_serial);
  IPAddress subnet = mrd_parse_ip_address(ETHER_SUBNET, a_serial);
  IPAddress dns = mrd_parse_ip_address(ETHER_DNS, a_serial);

  // パースエラーチェック
  if (local_ip == IPAddress(0, 0, 0, 0) ||
      gateway == IPAddress(0, 0, 0, 0) ||
      subnet == IPAddress(0, 0, 0, 0) ||
      dns == IPAddress(0, 0, 0, 0)) {
    a_serial.println("ERROR: Please check your IP address format in keys.h");
    return false;
  }

  // ネットワーク設定を検証
  if (!mrd_validate_network_config(local_ip, gateway, subnet, a_serial)) {
    a_serial.println("CRITICAL ERROR: Network configuration validation failed");
    return false;
  }

  // 標準Ethernetライブラリで初期化
  Ethernet.init(a_cs_pin);

  // 設定を適用 (引数のMACアドレスを使用)
  Ethernet.begin(mac_address, local_ip, dns, gateway, subnet);
  if (Ethernet.localIP() == local_ip) {
    a_serial.println("OK");
  } else {
    a_serial.println("FAILED: IP address mismatch");
    a_serial.println("Check hardware connections and network settings");
    return false;
  }

  // 結果を確認
  a_serial.print("Ether Local IP (Board's IP) => ");
  a_serial.println(Ethernet.localIP());
  a_serial.print("Ether Gateway IP (PC's IP) => ");
  a_serial.println(Ethernet.gatewayIP());

  // UDPを開始
  if (!udp_et.begin(UDP_RECV_PORT)) {
    a_serial.print("ERROR: Failed to start UDP on port ");
    a_serial.println(UDP_RECV_PORT);
    return false;
  }
  a_serial.print("UDP receive port: ");
  a_serial.println(UDP_RECV_PORT);
  return true;
}

/// @brief UDP経由でデータを受信しMeridim配列に格納する
/// @param a_meridim_bval バイト型のMeridim配列
/// @param a_len バイト型Meridim配列の長さ
/// @return 受信した場合はtrue, 受信しなかった場合はfalse
bool mrd_ether_udp_receive(byte *a_meridim_bval, int a_len) {
  int packet_size = udp_et.parsePacket();
  if (packet_size >= a_len) {
    udp_et.read(a_meridim_bval, a_len);
    return true;
  }
  return false; // バッファにデータなし
}

/// @brief Meridim配列データをUDP経由で指定IPとポートに送信する
/// @param a_meridim_bval バイト型のMeridim配列
/// @param a_len バイト型Meridim配列の長さ
/// @param a_send_ip 送信先IPアドレス
/// @return 完了時にtrueを返す
bool mrd_ether_udp_send(byte *a_meridim_bval, int a_len, IPAddress a_send_ip) {
  int result = udp_et.beginPacket(a_send_ip, UDP_SEND_PORT);
  if (result == 0) {
    return false; // パケット開始失敗
  }

  size_t bytes_written = udp_et.write(a_meridim_bval, a_len);
  if (bytes_written != a_len) {
    return false; // 書き込みサイズ不一致
  }

  result = udp_et.endPacket();
  return (result == 1); // 成功時は1を返す
}
