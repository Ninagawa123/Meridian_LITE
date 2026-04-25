#ifndef __MERIDIAN_ETHER_H__
#define __MERIDIAN_ETHER_H__

// ヘッダファイルの読み込み
#include "config.h"
#include "keys.h"

// ライブラリ導入 (標準Ethernetライブラリ)
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <SPI.h>

EthernetUDP udp_et; // Ethernet設定

//==================================================================================================
// Ethernet 関連の処理
//==================================================================================================

// 16進数文字かどうかをチェックする補助関数
bool isValidHexChar(char c) {
  return (c >= '0' && c <= '9') ||
         (c >= 'A' && c <= 'F') ||
         (c >= 'a' && c <= 'f');
}

// 16進数文字を数値に変換する補助関数
byte hexCharToByte(char c) {
  if (c >= '0' && c <= '9')
    return c - '0';
  if (c >= 'A' && c <= 'F')
    return c - 'A' + 10;
  if (c >= 'a' && c <= 'f')
    return c - 'a' + 10;
  return 0; // エラー（通常ここには到達しない）
}

// 文字列から余分なスペースを除去し、正規化する関数
String normalizeMacString(const char *macStr) {
  String normalized = "";
  for (int i = 0; macStr[i] != '\0'; i++) {
    if (macStr[i] != ' ' && macStr[i] != '\t') {
      normalized += macStr[i];
    }
  }
  return normalized;
}

// MACアドレス文字列をバイト配列にパースする関数（改良版）
bool parseMacAddress(const char *macStr, byte *macBytes) {
  if (!macStr || !macBytes) {
    return false;
  }

  // 余分なスペースを除去して正規化
  String normalizedMac = normalizeMacString(macStr);
  const char *cleanMac = normalizedMac.c_str();

  // 正規化後の長さチェック（17文字必要："XX:XX:XX:XX:XX:XX"）
  if (normalizedMac.length() != 17) {
    return false;
  }

  // 各バイトをパース
  for (int i = 0; i < 6; i++) {
    // 16進数文字の位置を計算
    int pos1 = i * 3;
    int pos2 = i * 3 + 1;
    int colonPos = i * 3 + 2;

    // 16進数文字の妥当性チェック
    if (!isValidHexChar(cleanMac[pos1]) || !isValidHexChar(cleanMac[pos2])) {
      return false;
    }

    // コロンのチェック（最後のバイト以外）
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

/// @brief IPアドレス設定の妥当性をチェックする
/// @param local_ip ローカルIP
/// @param gateway ゲートウェイIP
/// @param subnet サブネットマスク
/// @param a_serial エラー出力用シリアル
/// @return 妥当性チェック結果 (true: OK, false: NG)
bool mrd_validate_network_config(IPAddress local_ip, IPAddress gateway, IPAddress subnet, HardwareSerial &a_serial) {
  // ゼロIPアドレスチェック
  if (local_ip == IPAddress(0, 0, 0, 0)) {
    a_serial.println("ERROR: Local IP is invalid (0.0.0.0)");
    return false;
  }

  if (gateway == IPAddress(0, 0, 0, 0)) {
    a_serial.println("ERROR: Gateway IP is invalid (0.0.0.0)");
    return false;
  }

  if (subnet == IPAddress(0, 0, 0, 0)) {
    a_serial.println("ERROR: Subnet mask is invalid (0.0.0.0)");
    return false;
  }

  // ローカルIPとゲートウェイが同じサブネットにあるかチェック
  bool same_network = true;
  for (int i = 0; i < 4; i++) {
    if ((local_ip[i] & subnet[i]) != (gateway[i] & subnet[i])) {
      same_network = false;
      break;
    }
  }

  if (!same_network) {
    a_serial.println("WARNING: Local IP and Gateway are not in the same network");
    a_serial.print("Local IP network: ");
    for (int i = 0; i < 4; i++) {
      a_serial.print(local_ip[i] & subnet[i]);
      if (i < 3)
        a_serial.print(".");
    }
    a_serial.println();

    a_serial.print("Gateway network: ");
    for (int i = 0; i < 4; i++) {
      a_serial.print(gateway[i] & subnet[i]);
      if (i < 3)
        a_serial.print(".");
    }
    a_serial.println();
  }

  return true;
}

/// @brief 文字列形式のIPアドレスをIPAddressオブジェクトに変換する
/// @param ip_str IPアドレス文字列 (例: "192.168.1.1")
/// @param a_serial エラー出力用シリアル
/// @return 成功時はIPAddressオブジェクト、失敗時は IPAddress(0,0,0,0)
IPAddress mrd_parse_ip_address(const char *ip_str, HardwareSerial &a_serial) {
  uint8_t octets[4] = {0, 0, 0, 0};
  int octet_index = 0;
  int current_number = 0;
  bool has_digit = false;

  for (int i = 0; ip_str[i] != '\0'; i++) {
    char c = ip_str[i];

    if (c >= '0' && c <= '9') {
      // 数字の場合
      current_number = current_number * 10 + (c - '0');
      has_digit = true;

      // 範囲チェック (0-255)
      if (current_number > 255) {
        a_serial.print("ERROR Parsing IP: ");
        a_serial.println(ip_str);
        a_serial.println("Octet out of range (0-255), Ethernet initialization ABORTED.");
        return IPAddress(0, 0, 0, 0);
      }
    } else if (c == '.') {
      // ドット区切り文字の場合
      if (!has_digit) {
        a_serial.print("ERROR Parsing IP: ");
        a_serial.println(ip_str);
        a_serial.println("Invalid IP format, Ethernet initialization ABORTED.");
        return IPAddress(0, 0, 0, 0);
      }
      if (octet_index >= 4) {
        a_serial.print("ERROR Parsing IP: ");
        a_serial.println(ip_str);
        a_serial.println("Too many octets(expected 4), Ethernet initialization ABORTED.");
        return IPAddress(0, 0, 0, 0);
      }

      octets[octet_index] = current_number;
      octet_index++;
      current_number = 0;
      has_digit = false;
    } else if (c == ' ' || c == '\t') {
      // スペースやタブは無視（何もしない）
      continue;
    } else {
      // 無効な文字
      a_serial.print("ERROR Parsing IP: ");
      a_serial.println(ip_str);
      a_serial.println("Invalid character, Ethernet initialization ABORTED.");
      return IPAddress(0, 0, 0, 0);
    }
  }

  // 最後のオクテットを処理
  if (!has_digit) {
    a_serial.print("ERROR Parsing IP: ");
    a_serial.println(ip_str);
    a_serial.println("IP address ends without a digit, Ethernet initialization ABORTED.");
    return IPAddress(0, 0, 0, 0);
  }
  if (octet_index != 3) {
    a_serial.print("ERROR Parsing IP: ");
    a_serial.println(ip_str);
    a_serial.println("Incorrect number of octets(expected 4), Ethernet initialization ABORTED.");
    return IPAddress(0, 0, 0, 0);
  }

  octets[3] = current_number;
  return IPAddress(octets[0], octets[1], octets[2], octets[3]);
}

/// @brief Ethernetを初期化する（文字列IP設定対応版）
/// @param a_udp 使用するEthernetUDPのインスタンス
/// @param a_cs_pin W5500のCSピン番号
/// @param a_serial 出力先シリアルの指定.
/// @return 初期化に成功した場合はtrueを, 失敗した場合はfalseを返す.
bool mrd_ether_init(EthernetUDP &a_udp, int a_cs_pin, byte *mac_address, HardwareSerial &a_serial) {
  // MACアドレス表示
  a_serial.print("Wierd LAN MAC Address: ");
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

  // ネットワーク設定の妥当性チェック
  if (!mrd_validate_network_config(local_ip, gateway, subnet, a_serial)) {
    a_serial.println("CRITICAL ERROR: Network configuration validation failed");
    return false;
  }

  // 標準Ethernetライブラリでの初期化
  Ethernet.init(a_cs_pin);

  // 設定を適用（引数で受け取ったMACアドレスを使用）
  // Ethernet.begin(mac_address, local_ip, dns, gateway, subnet);

  Ethernet.begin(mac_address, local_ip, dns, gateway, subnet);
  if (Ethernet.localIP() == local_ip) {
    a_serial.println("OK");
  } else {
    a_serial.println("FAILED: IP address mismatch");
    a_serial.println("Check hardware connections and network settings");
    return false;
  }

  // 結果確認
  a_serial.print("Ether Local IP (Board's IP) => ");
  a_serial.println(Ethernet.localIP());
  a_serial.print("Ether Gateway IP (PC's IP) => ");
  a_serial.println(Ethernet.gatewayIP());

  // UDP開始
  if (!a_udp.begin(UDP_RECV_PORT)) {
    a_serial.print("ERROR: Failed to start UDP on port ");
    a_serial.println(UDP_RECV_PORT);
    return false;
  }
  a_serial.print("UDP receive port: ");
  a_serial.println(UDP_RECV_PORT);
  return true;
}

/// @brief 第一引数のMeridim配列にUDP経由でデータを受信, 格納する.
/// @param a_meridim_bval バイト型のMeridim配列
/// @param a_len バイト型のMeridim配列の長さ
/// @param a_udp 使用するEthernetUDPのインスタンス
/// @return 受信した場合はtrueを, 受信しなかった場合はfalseを返す.
bool mrd_ether_udp_receive(byte *a_meridim_bval, int a_len, EthernetUDP &a_udp) {
  int packet_size = a_udp.parsePacket();
  if (packet_size >= a_len) {
    a_udp.read(a_meridim_bval, a_len);

    // 受信元情報を取得（デバッグ用）
    IPAddress remote_ip = a_udp.remoteIP();
    uint16_t remote_port = a_udp.remotePort();

    return true;
  }
  return false; // バッファにデータがない
}

/// @brief 第一引数のMeridim配列のデータをUDP経由で指定したIPアドレス、ポートに送信する.
/// @param a_meridim_bval バイト型のMeridim配列
/// @param a_len バイト型のMeridim配列の長さ
/// @param a_udp 使用するEthernetUDPのインスタンス
/// @param a_send_ip 送信先IPアドレス
/// @param a_send_port 送信先ポート番号
/// @return 送信完了時にtrueを返す.
bool mrd_ether_udp_send(byte *a_meridim_bval, int a_len, EthernetUDP &a_udp, IPAddress a_send_ip, int a_send_port) {
  int result = a_udp.beginPacket(a_send_ip, a_send_port);
  if (result == 0) {
    return false; // パケット開始に失敗
  }

  size_t bytes_written = a_udp.write(a_meridim_bval, a_len);
  if (bytes_written != a_len) {
    return false; // 書き込みサイズが異なる
  }

  result = a_udp.endPacket();
  return (result == 1); // 成功時は1を返す
}

#endif // __MERIDIAN_ETHER_H__
