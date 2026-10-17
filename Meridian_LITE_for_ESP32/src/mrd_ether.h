#ifndef __MERIDIAN_ETHER_H__
#define __MERIDIAN_ETHER_H__

// ヘッダファイルの読み込み
#include "config.h"
#include "keys.h"
#include "main.h"

// ライブラリ導入 (標準Ethernetライブラリ)
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <SPI.h>

EthernetUDP udp_et; // Ethernet設定

//==================================================================================================
// Ethernet 関連の処理
//==================================================================================================

// 16進数文字かどうかをチェックする補助関数
bool mrd_ether_hex_char_is_valid(char a_c) {
  return (a_c >= '0' && a_c <= '9') ||
         (a_c >= 'A' && a_c <= 'F') ||
         (a_c >= 'a' && a_c <= 'f');
}

// 16進数文字を数値に変換する補助関数
byte mrd_ether_hex_char_to_byte(char a_c) {
  if (a_c >= '0' && a_c <= '9')
    return a_c - '0';
  if (a_c >= 'A' && a_c <= 'F')
    return a_c - 'A' + 10;
  if (a_c >= 'a' && a_c <= 'f')
    return a_c - 'a' + 10;
  return 0; // エラー（通常ここには到達しない）
}

// 文字列から余分なスペースを除去し、正規化する関数
String mrd_ether_mac_str_normalize(const char *a_mac_str) {
  String normalized_tmp = "";
  for (int i = 0; a_mac_str[i] != '\0'; i++) {
    if (a_mac_str[i] != ' ' && a_mac_str[i] != '\t') {
      normalized_tmp += a_mac_str[i];
    }
  }
  return normalized_tmp;
}

// MACアドレス文字列をバイト配列にパースする関数（改良版）
bool mrd_ether_mac_address_parse(const char *a_mac_str, byte *a_mac_bytes) {
  if (!a_mac_str || !a_mac_bytes) {
    return false;
  }

  // 余分なスペースを除去して正規化
  String normalized_mac_tmp = mrd_ether_mac_str_normalize(a_mac_str);
  const char *clean_mac_tmp = normalized_mac_tmp.c_str();

  // 正規化後の長さチェック（17文字必要："XX:XX:XX:XX:XX:XX"）
  if (normalized_mac_tmp.length() != 17) {
    return false;
  }

  // 各バイトをパース
  for (int i = 0; i < 6; i++) {
    // 16進数文字の位置を計算
    int pos1_tmp = i * 3;
    int pos2_tmp = i * 3 + 1;
    int colon_pos_tmp = i * 3 + 2;

    // 16進数文字の妥当性チェック
    if (!mrd_ether_hex_char_is_valid(clean_mac_tmp[pos1_tmp]) || !mrd_ether_hex_char_is_valid(clean_mac_tmp[pos2_tmp])) {
      return false;
    }

    // コロンのチェック（最後のバイト以外）
    if (i < 5 && clean_mac_tmp[colon_pos_tmp] != ':') {
      return false;
    }

    // 16進数文字をバイトに変換
    byte high_nibble_tmp = mrd_ether_hex_char_to_byte(clean_mac_tmp[pos1_tmp]);
    byte low_nibble_tmp = mrd_ether_hex_char_to_byte(clean_mac_tmp[pos2_tmp]);
    a_mac_bytes[i] = (high_nibble_tmp << 4) | low_nibble_tmp;
  }

  return true;
}

/// @brief IPアドレス設定の妥当性をチェックする
/// @param a_local_ip ローカルIP
/// @param a_gateway ゲートウェイIP
/// @param a_subnet サブネットマスク
/// @param a_serial エラー出力用シリアル
/// @return 妥当性チェック結果 (true: OK, false: NG)
bool mrd_network_config_validate(IPAddress a_local_ip, IPAddress a_gateway, IPAddress a_subnet, HardwareSerial &a_serial) {
  // ゼロIPアドレスチェック
  if (a_local_ip == IPAddress(0, 0, 0, 0)) {
    a_serial.println("ERROR: Local IP is invalid (0.0.0.0)");
    return false;
  }

  if (a_gateway == IPAddress(0, 0, 0, 0)) {
    a_serial.println("ERROR: Gateway IP is invalid (0.0.0.0)");
    return false;
  }

  if (a_subnet == IPAddress(0, 0, 0, 0)) {
    a_serial.println("ERROR: Subnet mask is invalid (0.0.0.0)");
    return false;
  }

  // ローカルIPとゲートウェイが同じサブネットにあるかチェック
  bool same_network_tmp = true;
  for (int i = 0; i < 4; i++) {
    if ((a_local_ip[i] & a_subnet[i]) != (a_gateway[i] & a_subnet[i])) {
      same_network_tmp = false;
      break;
    }
  }

  if (!same_network_tmp) {
    a_serial.println("WARNING: Local IP and Gateway are not in the same network");
    a_serial.print("Local IP network: ");
    for (int i = 0; i < 4; i++) {
      a_serial.print(a_local_ip[i] & a_subnet[i]);
      if (i < 3)
        a_serial.print(".");
    }
    a_serial.println();

    a_serial.print("Gateway network: ");
    for (int i = 0; i < 4; i++) {
      a_serial.print(a_gateway[i] & a_subnet[i]);
      if (i < 3)
        a_serial.print(".");
    }
    a_serial.println();
  }

  return true;
}

/// @brief 文字列形式のIPアドレスをIPAddressオブジェクトに変換する
/// @param a_ip_str IPアドレス文字列 (例: "192.168.1.1")
/// @param a_serial エラー出力用シリアル
/// @return 成功時はIPAddressオブジェクト、失敗時は IPAddress(0,0,0,0)
IPAddress mrd_ip_address_parse(const char *a_ip_str, HardwareSerial &a_serial) {
  uint8_t octets_tmp[4] = {0, 0, 0, 0};
  int octet_index_tmp = 0;
  int current_number_tmp = 0;
  bool has_digit_tmp = false;

  for (int i = 0; a_ip_str[i] != '\0'; i++) {
    char c_tmp = a_ip_str[i];

    if (c_tmp >= '0' && c_tmp <= '9') {
      // 数字の場合
      current_number_tmp = current_number_tmp * 10 + (c_tmp - '0');
      has_digit_tmp = true;

      // 範囲チェック (0-255)
      if (current_number_tmp > 255) {
        a_serial.print("ERROR Parsing IP: ");
        a_serial.println(a_ip_str);
        a_serial.println("Octet out of range (0-255), Ethernet initialization ABORTED.");
        return IPAddress(0, 0, 0, 0);
      }
    } else if (c_tmp == '.') {
      // ドット区切り文字の場合
      if (!has_digit_tmp) {
        a_serial.print("ERROR Parsing IP: ");
        a_serial.println(a_ip_str);
        a_serial.println("Invalid IP format, Ethernet initialization ABORTED.");
        return IPAddress(0, 0, 0, 0);
      }
      if (octet_index_tmp >= 4) {
        a_serial.print("ERROR Parsing IP: ");
        a_serial.println(a_ip_str);
        a_serial.println("Too many octets(expected 4), Ethernet initialization ABORTED.");
        return IPAddress(0, 0, 0, 0);
      }

      octets_tmp[octet_index_tmp] = current_number_tmp;
      octet_index_tmp++;
      current_number_tmp = 0;
      has_digit_tmp = false;
    } else if (c_tmp == ' ' || c_tmp == '\t') {
      // スペースやタブは無視（何もしない）
      continue;
    } else {
      // 無効な文字
      a_serial.print("ERROR Parsing IP: ");
      a_serial.println(a_ip_str);
      a_serial.println("Invalid character, Ethernet initialization ABORTED.");
      return IPAddress(0, 0, 0, 0);
    }
  }

  // 最後のオクテットを処理
  if (!has_digit_tmp) {
    a_serial.print("ERROR Parsing IP: ");
    a_serial.println(a_ip_str);
    a_serial.println("IP address ends without a digit, Ethernet initialization ABORTED.");
    return IPAddress(0, 0, 0, 0);
  }
  if (octet_index_tmp != 3) {
    a_serial.print("ERROR Parsing IP: ");
    a_serial.println(a_ip_str);
    a_serial.println("Incorrect number of octets(expected 4), Ethernet initialization ABORTED.");
    return IPAddress(0, 0, 0, 0);
  }

  octets_tmp[3] = current_number_tmp;
  return IPAddress(octets_tmp[0], octets_tmp[1], octets_tmp[2], octets_tmp[3]);
}

/// @brief Ethernetを初期化する（文字列IP設定対応版）
/// @param a_udp 使用するEthernetUDPのインスタンス
/// @param a_cs_pin W5500のCSピン番号
/// @param a_serial 出力先シリアルの指定.
/// @return 初期化に成功した場合はtrueを, 失敗した場合はfalseを返す.
bool mrd_ether_init(EthernetUDP &a_udp, int a_cs_pin, byte *a_mac_address, HardwareSerial &a_serial) {
  // MACアドレス表示
  a_serial.print("Wired LAN MAC Address: ");
  for (int i = 0; i < 6; i++) {
    if (a_mac_address[i] < 16)
      a_serial.print("0");
    a_serial.print(a_mac_address[i], HEX);
    if (i < 5)
      a_serial.print(":");
  }
  a_serial.println();
  a_serial.print("Initializing Ethernet... ");

  // IPアドレス文字列をパース
  IPAddress local_ip_tmp = mrd_ip_address_parse(cfg.ether_local_ip, a_serial);
  IPAddress gateway_tmp = mrd_ip_address_parse(cfg.ether_gateway, a_serial);
  IPAddress subnet_tmp = mrd_ip_address_parse(cfg.ether_subnet, a_serial);
  IPAddress dns_tmp = mrd_ip_address_parse(cfg.ether_dns, a_serial);

  // パースエラーチェック
  if (local_ip_tmp == IPAddress(0, 0, 0, 0) ||
      gateway_tmp == IPAddress(0, 0, 0, 0) ||
      subnet_tmp == IPAddress(0, 0, 0, 0) ||
      dns_tmp == IPAddress(0, 0, 0, 0)) {
    a_serial.println("ERROR: Please check your IP address format in keys.h");
    return false;
  }

  // ネットワーク設定の妥当性チェック
  if (!mrd_network_config_validate(local_ip_tmp, gateway_tmp, subnet_tmp, a_serial)) {
    a_serial.println("CRITICAL ERROR: Network configuration validation failed");
    return false;
  }

  // 標準Ethernetライブラリでの初期化
  Ethernet.init(a_cs_pin);

  // 設定を適用（引数で受け取ったMACアドレスを使用）
  Ethernet.begin(a_mac_address, local_ip_tmp, dns_tmp, gateway_tmp, subnet_tmp);
  if (Ethernet.localIP() == local_ip_tmp) {
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
  if (!a_udp.begin(cfg.udp_recv_port)) {
    a_serial.print("ERROR: Failed to start UDP on port ");
    a_serial.println(cfg.udp_recv_port);
    return false;
  }
  a_serial.print("UDP receive port: ");
  a_serial.println(cfg.udp_recv_port);
  return true;
}

/// @brief 第一引数のMeridim配列にUDP経由でデータを受信, 格納する.
/// @param a_meridim_bval バイト型のMeridim配列
/// @param a_len バイト型のMeridim配列の長さ
/// @param a_udp 使用するEthernetUDPのインスタンス
/// @return 受信した場合はtrueを, 受信しなかった場合はfalseを返す.
bool mrd_ether_udp_receive(byte *a_meridim_bval, int a_len, EthernetUDP &a_udp) {
  bool rcvd_tmp = false;
  int packet_size_tmp;
  while ((packet_size_tmp = a_udp.parsePacket()) > 0) { // 溜まっているパケットをすべて読み, 最新の1つを使う(WiFiと同じ)
    if (packet_size_tmp >= a_len) {
      a_udp.read(a_meridim_bval, a_len); // 読み残しは次のparsePacketで捨てられる
      rcvd_tmp = true;
    }
  }
  return rcvd_tmp;
}

/// @brief 第一引数のMeridim配列のデータをUDP経由で指定したIPアドレス、ポートに送信する.
/// @param a_meridim_bval バイト型のMeridim配列
/// @param a_len バイト型のMeridim配列の長さ
/// @param a_udp 使用するEthernetUDPのインスタンス
/// @param a_send_ip 送信先IPアドレス
/// @param a_send_port 送信先ポート番号
/// @return 送信完了時にtrueを返す.
bool mrd_ether_udp_send(byte *a_meridim_bval, int a_len, EthernetUDP &a_udp, IPAddress a_send_ip, int a_send_port) {
  int result_tmp = a_udp.beginPacket(a_send_ip, a_send_port);
  if (result_tmp == 0) {
    return false; // パケット開始に失敗
  }

  size_t bytes_written_tmp = a_udp.write(a_meridim_bval, a_len);
  if (bytes_written_tmp != a_len) {
    return false; // 書き込みサイズが異なる
  }

  result_tmp = a_udp.endPacket();
  return (result_tmp == 1); // 成功時は1を返す
}

#endif // __MERIDIAN_ETHER_H__
