// mrd_util.cpp
// ユーティリティ関数の実装

// ヘッダファイルの読み込み
#include "mrd_util.h"

// ライブラリ導入

//==================================================================================================
// ユーティリティ関数
//==================================================================================================

/// @brief 配列内で非ゼロ値を持つ最大インデックス+1を取得する (ループ回数用)
/// @param a_arr 配列
/// @param a_size 配列の長さ
/// @return 非ゼロ値を持つ最大インデックス+1. 全てゼロの場合は1を返す.
int mrd_max_used_index(const int a_arr[], int a_size) {
  int max_index_tmp = 0;
  for (int i = 0; i < a_size; ++i) {
    if (a_arr[i] != 0) {
      max_index_tmp = i;
    }
  }
  return max_index_tmp + 1;
}

//------------------------------------------------------------------------------------
//  表示フォーマット
//------------------------------------------------------------------------------------

/// @brief 10のべき乗を整数演算で計算する (精度問題回避用)
/// @param exp 指数 (0以上)
/// @return 10^exp の整数値
static int int_pow10(int exp) {
  int result = 1;
  for (int i = 0; i < exp; i++) {
    result *= 10;
  }
  return result;
}

/// @brief シリアルモニタ表示用に数値をパディングする
/// @param num 表示する値
/// @param total_width 桁数
/// @param frac_width 小数点以下の桁数 (0で小数点なし)
/// @param show_plus +符号を表示するか
/// @return フォーマット済み文字列
String mrd_pddstr(float num, int total_width, int frac_width, bool show_plus) {
  char buf[30];
  char sign = (num < 0) ? '-' : (show_plus ? '+' : '\0');
  if (num < 0)
    num = -num;

  // 小数点あり/なしの処理 (バッファオーバーフロー防止にsnprintfを使用)
  if (frac_width) {
    int multiplier = int_pow10(frac_width);
    if (sign)
      snprintf(buf, sizeof(buf), "%c%d.%0*d", sign, (int)num, frac_width, (int)((num - (int)num) * multiplier + 0.5));
    else
      snprintf(buf, sizeof(buf), "%d.%0*d", (int)num, frac_width, (int)((num - (int)num) * multiplier + 0.5));
  } else {
    if (sign)
      snprintf(buf, sizeof(buf), "%c%d", sign, (int)(num + 0.5));
    else
      snprintf(buf, sizeof(buf), "%d", (int)(num + 0.5));
  }

  // パディング
  String result = "";
  int pad = total_width - strlen(buf);
  for (int i = 0; i < pad; i++)
    result += ' ';
  result += buf;
  return result;
}

//------------------------------------------------------------------------------------
//  タイムアウト監視タイマー
//------------------------------------------------------------------------------------

// タイマー状態用の静的変数
static unsigned long timeout_start = 0;
static bool flg_timer_started = false; // タイマー開始状態フラグ

/// @brief 指定ミリ秒のタイムアウトを監視する. mrd_timeout_resetと併用.
/// @param a_limit タイムアウト時間 (ms)
/// @return タイムアウト時はtrue
bool mrd_timeout_check(unsigned long a_limit) {
  // タイマーが開始されていない場合, 現在時刻を記録してタイマーを開始
  if (!flg_timer_started) {
    timeout_start = millis();
    flg_timer_started = true; // タイムアウト監視フラグをセット
  }

  unsigned long current_time = millis(); // 現在時刻を取得

  if (current_time - timeout_start >= a_limit) { // 指定時間が経過したか確認
    flg_timer_started = false;                   // タイムアウト監視フラグをクリア
    return true;                                 // 時間経過時はtrueを返す
  }

  return false; // 時間未経過時はfalseを返す
}

/// @brief タイムアウト監視フラグをリセットする. mrd_timeout_checkと併用.
void mrd_timeout_reset() {
  flg_timer_started = false; // 次回呼び出し用にタイマーをリセット
}

/// @brief enum (L, R, C) から文字列を取得する
/// @param a_line enum型 enum UartLine
/// @return enumの内容に応じて "L", "R", "C" の文字列
const char *mrd_get_line_name(UartLine a_line) {
  switch (a_line) {
  case L:
    return "L";
  case R:
    return "R";
  case C:
    return "C";
  default:
    return "Unknown";
  }
}

//------------------------------------------------------------------------------------
//  meriput / meridimデータ書き込み
//------------------------------------------------------------------------------------

/// @brief エラーメッセージとLED点滅, タイムアウト後に再起動
/// @param a_led エラー通知LED用のピン番号
/// @param a_msg エラーメッセージ
/// @param a_serial 出力シリアル
/// @param a_restart_ms 再起動までのタイムアウト (ms, デフォルト10000ms, 0で再起動しない)
void mrd_error_stop(int a_led, String a_msg, HardwareSerial &a_serial, unsigned long a_restart_ms) {
  a_serial.println(a_msg);
  a_serial.println("System will restart in " + String(a_restart_ms / 1000) + " seconds...");

  unsigned long start_time = millis();
  while (a_restart_ms == 0 || (millis() - start_time < a_restart_ms)) {
    digitalWrite(a_led, HIGH);
    delay(250);
    digitalWrite(a_led, LOW);
    delay(250);
  }

  // ESP32を再起動
  a_serial.println("Restarting...");
  delay(100);
  ESP.restart();
}

/// @brief meridim配列のチェックサムを計算して[len-1]に書き込む
/// @param a_meridim Meridim配列共用体. 参照渡し.
/// @param len 配列の長さ (デフォルト90)
/// @return 常にtrueを返す
bool mrd_meriput90_cksm(Meridim90Union &a_meridim, int len) {
  int a_cksm = 0;
  for (int i = 0; i < len - 1; i++) {
    a_cksm += int(a_meridim.sval[i]);
  }
  a_meridim.sval[len - 1] = short(~a_cksm);
  return true;
}

//------------------------------------------------------------------------------------
//  ネットワークヘルパー関数 (WiFi/Ethernet共通)
//------------------------------------------------------------------------------------

/// @brief IPアドレス設定を検証する
/// @param local_ip ローカルIP
/// @param gateway ゲートウェイIP
/// @param subnet サブネットマスク
/// @param a_serial エラー出力用シリアル
/// @return 検証結果 (true: OK, false: NG)
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

  // ローカルIPとゲートウェイが同じサブネットかチェック
  bool same_network = true;
  for (int i = 0; i < 4; i++) {
    if ((local_ip[i] & subnet[i]) != (gateway[i] & subnet[i])) {
      same_network = false;
      break;
    }
  }

  if (!same_network) {
    a_serial.println("WARNING: Local IP and Gateway are not in the same network");
  }

  return true;
}

/// @brief IPアドレス文字列をIPAddressオブジェクトに変換する
/// @param ip_str IPアドレス文字列 (例: "192.168.1.1")
/// @param a_serial エラー出力用シリアル
/// @return 成功時はIPAddressオブジェクト, 失敗時はIPAddress(0,0,0,0)
IPAddress mrd_parse_ip_address(const char *ip_str, HardwareSerial &a_serial) {
  uint8_t octets[4] = {0, 0, 0, 0};
  int octet_index = 0;
  int current_number = 0;
  bool has_digit = false;

  for (int i = 0; ip_str[i] != '\0'; i++) {
    char c = ip_str[i];

    if (c >= '0' && c <= '9') {
      current_number = current_number * 10 + (c - '0');
      has_digit = true;

      if (current_number > 255) {
        a_serial.print("ERROR Parsing IP: ");
        a_serial.println(ip_str);
        a_serial.println("Octet out of range (0-255)");
        return IPAddress(0, 0, 0, 0);
      }
    } else if (c == '.') {
      if (!has_digit || octet_index >= 4) {
        a_serial.print("ERROR Parsing IP: ");
        a_serial.println(ip_str);
        return IPAddress(0, 0, 0, 0);
      }

      octets[octet_index] = current_number;
      octet_index++;
      current_number = 0;
      has_digit = false;
    } else if (c == ' ' || c == '\t') {
      continue;
    } else {
      a_serial.print("ERROR Parsing IP: ");
      a_serial.println(ip_str);
      return IPAddress(0, 0, 0, 0);
    }
  }

  if (!has_digit || octet_index != 3) {
    a_serial.print("ERROR Parsing IP: ");
    a_serial.println(ip_str);
    return IPAddress(0, 0, 0, 0);
  }

  octets[3] = current_number;
  return IPAddress(octets[0], octets[1], octets[2], octets[3]);
}
