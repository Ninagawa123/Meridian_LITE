#ifndef __MERIDIAN_EEPROM_H__
#define __MERIDIAN_EEPROM_H__

// ヘッダファイルの読み込み
#include "config.h"
#include "main.h"
#include "mrd_util.h"

// ライブラリ導入
#include <EEPROM.h>

//==================================================================================================
//  EEPROMの配置
//==================================================================================================
// 1ページ = Meridim90 1つ分(90語, 180byte). ページ単位で Meridian 通信(10200+p / 10300+p)で読み書きできる.
//   語[0]: ページの印(MRD_EEP_MARK + ページ番号)  語[1]: 配置の版  語[2]〜[87]: データ  語[88]: 予備  語[89]: チェックサム
//   通信では [0][1][88][89] がマスターコマンド・連番・エラー・チェックサムになるため, データは [2]〜[87] に置く.
// ページの内容(仕様書16章):
//   P0 システム・動作モード, P1 サーボのスロット(Meridian_console と同じ形), P2 スロットの拡張(種類, 8bitのID),
//   P3 サーボの系統と共通設定, P4 PWM, P5 ネットワーク(keys.h), P6〜 予備.
// 値はint16. 小数は決まった倍率の整数, 32bitの値は2語(下位, 上位), 文字列・IP・MACはbyteの並びのまま入れる.

const int MRD_EEP_PAGE_WORDS = 90;                                // 1ページの語数
const int MRD_EEP_PAGES = EEPROM_SIZE / (MRD_EEP_PAGE_WORDS * 2); // ページ数
const int MRD_EEP_DATA_FIRST = 2;                                 // データの最初の語
const int MRD_EEP_DATA_LAST = 87;                                 // データの最後の語
const int16_t MRD_EEP_MARK = 0x4D00;                              // ページの印("M" + ページ番号)
const int16_t MRD_EEP_VERSION = 1;                                // 配置の版
static_assert(EEPROM_SIZE % (MRD_EEP_PAGE_WORDS * 2) == 0, "EEPROM_SIZE must be a multiple of 180 (one page)");

enum MrdEepPage {         // 使っているページ(EEPROM_LOAD のビットの位置と同じ)
  MRD_EEP_P_SYSTEM = 0,   // システム・動作モード
  MRD_EEP_P_SERVO = 1,    // サーボのスロット
  MRD_EEP_P_SLOT_EXT = 2, // スロットの拡張
  MRD_EEP_P_LINE = 3,     // サーボの系統と共通設定
  MRD_EEP_P_PWM = 4,      // PWM
  MRD_EEP_P_NET = 5,      // ネットワーク
  MRD_EEP_P_USED = 6      // 使っているページの数
};
static_assert(MRD_EEP_PAGES >= MRD_EEP_P_USED, "EEPROM_SIZE must hold the used pages (6 x 180 bytes or more)");

// P5 の WiFi のSSIDとパスワード
const int MRD_EEP_NET_SSID = 25; // SSID(32byte = 16語)
const int MRD_EEP_NET_PASS = 41; // パスワード(64byte = 32語)

// EEPROM読み書き用共用体
typedef union {
  uint8_t bval[EEPROM_SIZE];                          // 1バイト単位
  int16_t saval[MRD_EEP_PAGES][MRD_EEP_PAGE_WORDS];   // ページ, 語(short型)
  uint16_t usaval[MRD_EEP_PAGES][MRD_EEP_PAGE_WORDS]; // ページ, 語(unsigned short型)
} EepromUnion;

//------------------------------------------------------------------------------------
//  項目表(ページの語と設定値の対応)
//------------------------------------------------------------------------------------

enum MrdEepKind {   // 項目の値の形
  MRD_EEP_INT,      // int. 1語
  MRD_EEP_U16,      // int(0〜65535). 1語
  MRD_EEP_U32,      // unsigned long. 2語(下位, 上位)
  MRD_EEP_FLOAT,    // float × scale を整数にして1語
  MRD_EEP_INTS,     // int の配列(scale 個). 1つ1語
  MRD_EEP_STR,      // 文字列(scale byte). 足りない分は0
  MRD_EEP_BYTES_IP, // "a.b.c.d" の文字列 ↔ 4byte(2語)
  MRD_EEP_BYTES_MAC // "XX:XX:XX:XX:XX:XX" の文字列 ↔ 6byte(3語)
};

struct MrdEepField {
  uint8_t page;  // ページ
  uint8_t word;  // 最初の語
  uint8_t kind;  // 値の形(MrdEepKind)
  void *ptr;     // 設定値の場所
  int16_t scale; // FLOAT: 倍率, INTS: 個数, STR: byte数
  long min;      // 読み込むときの範囲(語の値. FLOAT は倍率をかけた値). 範囲外なら config の値のまま
  long max;
};

// 仕様書16章の表と同じ. 項目を足すときは予備の語を使い, 既存の語の位置は変えない.
const MrdEepField MRD_EEP_FIELDS[] = {
    // P0 システム・動作モード
    {MRD_EEP_P_SYSTEM, 2, MRD_EEP_INT, &cfg.frame_duration, 0, 1, 100},
    {MRD_EEP_P_SYSTEM, 3, MRD_EEP_INT, &cfg.charge_time, 0, 0, 10000},
    {MRD_EEP_P_SYSTEM, 4, MRD_EEP_INT, &cfg.mount_sd, 0, 0, 1},
    {MRD_EEP_P_SYSTEM, 5, MRD_EEP_INT, &cfg.mount_imuahrs, 0, 0, 3},
    {MRD_EEP_P_SYSTEM, 6, MRD_EEP_INT, &cfg.mount_pad, 0, 0, 6},
    {MRD_EEP_P_SYSTEM, 7, MRD_EEP_INT, &cfg.mode_esp32_standalone, 0, 0, 1},
    {MRD_EEP_P_SYSTEM, 8, MRD_EEP_INT, &cfg.mode_udp_receive, 0, 0, 1},
    {MRD_EEP_P_SYSTEM, 9, MRD_EEP_INT, &cfg.mode_udp_send, 0, 0, 1},
    {MRD_EEP_P_SYSTEM, 10, MRD_EEP_INT, &cfg.udp_timeout_us, 0, 0, 30000},
    {MRD_EEP_P_SYSTEM, 11, MRD_EEP_INT, &cfg.udp_poll_us, 0, 1, 10000},
    {MRD_EEP_P_SYSTEM, 12, MRD_EEP_INT, &cfg.udp_failsafe_ms, 0, 0, 32767},
    {MRD_EEP_P_SYSTEM, 13, MRD_EEP_FLOAT, &cfg.udp_self_speed, 10, 0, 32767},
    {MRD_EEP_P_SYSTEM, 14, MRD_EEP_INT, &cfg.check_sd_rw, 0, 0, 1},
    {MRD_EEP_P_SYSTEM, 15, MRD_EEP_INT, &cfg.monitor_frame_delay, 0, 0, 1},
    {MRD_EEP_P_SYSTEM, 16, MRD_EEP_INT, &cfg.monitor_flow, 0, 0, 1},
    {MRD_EEP_P_SYSTEM, 17, MRD_EEP_INT, &cfg.monitor_err_all, 0, 0, 1},
    {MRD_EEP_P_SYSTEM, 18, MRD_EEP_INT, &cfg.monitor_seq, 0, 0, 1},
    {MRD_EEP_P_SYSTEM, 19, MRD_EEP_INT, &cfg.monitor_udp_timeout, 0, 0, 1},
    {MRD_EEP_P_SYSTEM, 20, MRD_EEP_INT, &cfg.monitor_suppress_duration, 0, 0, 32767},
    {MRD_EEP_P_SYSTEM, 21, MRD_EEP_U32, &cfg.i2c0_speed, 0, 10000, 1000000},
    {MRD_EEP_P_SYSTEM, 23, MRD_EEP_INT, &cfg.imuahrs_interval, 0, 1, 1000},
    {MRD_EEP_P_SYSTEM, 24, MRD_EEP_U32, &cfg.serial_pc_bps, 0, 9600, 5000000},
    {MRD_EEP_P_SYSTEM, 26, MRD_EEP_INT, &cfg.pad_init_timeout, 0, 0, 32767},
    {MRD_EEP_P_SYSTEM, 27, MRD_EEP_INT, &cfg.pad_interval, 0, 1, 1000},
    {MRD_EEP_P_SYSTEM, 28, MRD_EEP_INT, &cfg.pad_button_marge, 0, 0, 1},
    {MRD_EEP_P_SYSTEM, 29, MRD_EEP_INT, &cfg.pad_generalize, 0, 0, 1},
    {MRD_EEP_P_SYSTEM, 30, MRD_EEP_U32, &cfg.pad_sbdbt_bps, 0, 1200, 230400},
    {MRD_EEP_P_SYSTEM, 32, MRD_EEP_INT, &cfg.pad_sbdbt_timeout, 0, 0, 32767},
    {MRD_EEP_P_SYSTEM, 33, MRD_EEP_INT, &cfg.pad_blueretro_i2c, 0, 0, 1},
    // P3 サーボの系統と共通設定
    {MRD_EEP_P_LINE, 2, MRD_EEP_INT, &cfg.mount_servo_type_l, 0, 0, 127},
    {MRD_EEP_P_LINE, 3, MRD_EEP_INT, &cfg.mount_servo_type_r, 0, 0, 127},
    {MRD_EEP_P_LINE, 4, MRD_EEP_INT, &cfg.mount_servo_type_l2, 0, 0, 127},
    {MRD_EEP_P_LINE, 5, MRD_EEP_INT, &cfg.mount_servo_type_r2, 0, 0, 127},
    {MRD_EEP_P_LINE, 6, MRD_EEP_U32, &cfg.servo_baudrate_l, 0, 1200, 5000000},
    {MRD_EEP_P_LINE, 8, MRD_EEP_U32, &cfg.servo_baudrate_r, 0, 1200, 5000000},
    {MRD_EEP_P_LINE, 10, MRD_EEP_U32, &cfg.servo_baudrate_l2, 0, 1200, 5000000},
    {MRD_EEP_P_LINE, 12, MRD_EEP_U32, &cfg.servo_baudrate_r2, 0, 1200, 5000000},
    {MRD_EEP_P_LINE, 14, MRD_EEP_INT, &cfg.servo_timeout_l, 0, 1, 100},
    {MRD_EEP_P_LINE, 15, MRD_EEP_INT, &cfg.servo_timeout_r, 0, 1, 100},
    {MRD_EEP_P_LINE, 16, MRD_EEP_INT, &cfg.servo_echo_l, 0, 0, 2},
    {MRD_EEP_P_LINE, 17, MRD_EEP_INT, &cfg.servo_echo_r, 0, 0, 2},
    {MRD_EEP_P_LINE, 18, MRD_EEP_INT, &cfg.servo_tx_release, 0, 0, 1},
    {MRD_EEP_P_LINE, 19, MRD_EEP_INT, &cfg.servo_lost_err_wait, 0, 1, 100},
    {MRD_EEP_P_LINE, 20, MRD_EEP_FLOAT, &cfg.servo_step_max, 100, 0, 32767},
    {MRD_EEP_P_LINE, 21, MRD_EEP_INT, &cfg.servo_unmount_reply, 0, 0, 1},
    {MRD_EEP_P_LINE, 22, MRD_EEP_INT, &cfg.servo_motion_frames_max, 0, 0, 32767},
    {MRD_EEP_P_LINE, 23, MRD_EEP_FLOAT, &cfg.servo_ftc_sts_range, 10, 1, 32767},
    {MRD_EEP_P_LINE, 24, MRD_EEP_FLOAT, &cfg.servo_ftc_scs_range, 10, 1, 32767},
    {MRD_EEP_P_LINE, 25, MRD_EEP_INT, &cfg.servo_ftc_scs_speed, 0, 0, 32767},
    {MRD_EEP_P_LINE, 26, MRD_EEP_INT, &cfg.servo_ftc_fix_return_delay, 0, 0, 1},
    {MRD_EEP_P_LINE, 27, MRD_EEP_INT, &cfg.servo_dxl_fix_return_delay, 0, 0, 1},
    {MRD_EEP_P_LINE, 28, MRD_EEP_INT, &cfg.servo_xbus_angle180, 0, 0, 1},
    {MRD_EEP_P_LINE, 29, MRD_EEP_INT, &cfg.servo_htc_per_frame, 0, 1, 15},
    {MRD_EEP_P_LINE, 30, MRD_EEP_FLOAT, &cfg.servo_htc_us_per_deg, 100, 1, 32767},
    {MRD_EEP_P_LINE, 31, MRD_EEP_INT, &cfg.servo_htc_speed, 0, 1, 255},
    {MRD_EEP_P_LINE, 32, MRD_EEP_INT, &cfg.servo_ftb_read_per_frame, 0, 1, 15},
    // P4 PWM
    {MRD_EEP_P_PWM, 2, MRD_EEP_INT, &cfg.pwm_pin_mode, 0, 0, 2},
    {MRD_EEP_P_PWM, 3, MRD_EEP_FLOAT, &cfg.pwm_pulse_center, 1, 0, 32767},
    {MRD_EEP_P_PWM, 4, MRD_EEP_FLOAT, &cfg.pwm_us_per_deg, 1000, 1, 32767},
    {MRD_EEP_P_PWM, 5, MRD_EEP_FLOAT, &cfg.pwm_pulse_min, 1, 0, 32767},
    {MRD_EEP_P_PWM, 6, MRD_EEP_FLOAT, &cfg.pwm_pulse_max, 1, 0, 32767},
    {MRD_EEP_P_PWM, 7, MRD_EEP_U32, &cfg.i2c1_speed, 0, 10000, 1000000},
    {MRD_EEP_P_PWM, 9, MRD_EEP_INT, &cfg.pca9685_num, 0, 0, 2},
    {MRD_EEP_P_PWM, 10, MRD_EEP_INT, &cfg.pca9685_addr_0, 0, 0, 127},
    {MRD_EEP_P_PWM, 11, MRD_EEP_INT, &cfg.pca9685_addr_1, 0, 0, 127},
    {MRD_EEP_P_PWM, 12, MRD_EEP_U32, &cfg.pca9685_osc_hz, 0, 1000000, 50000000},
    {MRD_EEP_P_PWM, 14, MRD_EEP_INTS, PWM_PIN_SLOT, 2, -1, 114},
    {MRD_EEP_P_PWM, 16, MRD_EEP_INTS, &PCA9685_SLOT[0][0], 32, -1, 114},
    // P5 ネットワーク
    {MRD_EEP_P_NET, 2, MRD_EEP_INT, &cfg.mode_ether, 0, 0, 1},
    {MRD_EEP_P_NET, 3, MRD_EEP_INT, &cfg.mode_fixed_ip, 0, 0, 1},
    {MRD_EEP_P_NET, 4, MRD_EEP_U16, &cfg.udp_send_port, 0, 1, 65535},
    {MRD_EEP_P_NET, 5, MRD_EEP_U16, &cfg.udp_recv_port, 0, 1, 65535},
    {MRD_EEP_P_NET, 6, MRD_EEP_BYTES_IP, cfg.wifi_send_ip, 0, 0, 0},
    {MRD_EEP_P_NET, 8, MRD_EEP_BYTES_IP, cfg.fixed_ip_addr, 0, 0, 0},
    {MRD_EEP_P_NET, 10, MRD_EEP_BYTES_IP, cfg.fixed_ip_gateway, 0, 0, 0},
    {MRD_EEP_P_NET, 12, MRD_EEP_BYTES_IP, cfg.fixed_ip_subnet, 0, 0, 0},
    {MRD_EEP_P_NET, 14, MRD_EEP_BYTES_IP, cfg.ether_local_ip, 0, 0, 0},
    {MRD_EEP_P_NET, 16, MRD_EEP_BYTES_IP, cfg.ether_gateway, 0, 0, 0},
    {MRD_EEP_P_NET, 18, MRD_EEP_BYTES_IP, cfg.ether_subnet, 0, 0, 0},
    {MRD_EEP_P_NET, 20, MRD_EEP_BYTES_IP, cfg.ether_dns, 0, 0, 0},
    {MRD_EEP_P_NET, 22, MRD_EEP_BYTES_MAC, cfg.ether_mac, 0, 0, 0},
    {MRD_EEP_P_NET, MRD_EEP_NET_SSID, MRD_EEP_STR, cfg.wifi_ap_ssid, 32, 0, 0},
    {MRD_EEP_P_NET, MRD_EEP_NET_PASS, MRD_EEP_STR, cfg.wifi_ap_pass, 64, 0, 0},
};
const int MRD_EEP_FIELD_NUM = sizeof(MRD_EEP_FIELDS) / sizeof(MRD_EEP_FIELDS[0]);

//==================================================================================================
//  ページの作成と検査
//==================================================================================================

/// @brief "a.b.c.d"(区切り '.', 10進)や "XX:XX:..."(区切り ':', 16進)の文字列を byte の並びにする.
/// @return 書式どおりならtrue. 誤りなら a_out はすべて0.
bool mrd_eeprom_text_to_bytes(const char *a_text, char a_sep, int a_base, uint8_t *a_out, int a_num) {
  int k = 0;
  long val_tmp = -1; // -1: この区切りでまだ数字がない
  for (const char *c_tmp = a_text;; c_tmp++) {
    int d_tmp = -1;
    if (*c_tmp >= '0' && *c_tmp <= '9') {
      d_tmp = *c_tmp - '0';
    } else if (a_base == 16 && *c_tmp >= 'a' && *c_tmp <= 'f') {
      d_tmp = *c_tmp - 'a' + 10;
    } else if (a_base == 16 && *c_tmp >= 'A' && *c_tmp <= 'F') {
      d_tmp = *c_tmp - 'A' + 10;
    }
    if (d_tmp >= 0) {
      val_tmp = (val_tmp < 0 ? 0 : val_tmp) * a_base + d_tmp;
      if (val_tmp > 255) {
        break;
      }
      continue;
    }
    if ((*c_tmp == a_sep || *c_tmp == '\0') && val_tmp >= 0 && k < a_num) {
      a_out[k++] = uint8_t(val_tmp);
      val_tmp = -1;
      if (*c_tmp == '\0') {
        if (k == a_num) {
          return true;
        }
        break;
      }
      continue;
    }
    break;
  }
  memset(a_out, 0, a_num);
  return false;
}

/// @brief byte の並びを "a.b.c.d"(10進) か "XX:XX:..."(16進) の文字列にする(a_out は a_num × 4 byte 以上).
void mrd_eeprom_bytes_to_text(const uint8_t *a_in, int a_num, char a_sep, bool a_hex, char *a_out) {
  char *out_tmp = a_out;
  for (int k = 0; k < a_num; k++) {
    out_tmp += snprintf(out_tmp, 4, a_hex ? "%02X" : "%u", a_in[k]); // 1byte は3文字まで
    if (k < a_num - 1) {
      *out_tmp++ = a_sep;
    }
  }
  *out_tmp = '\0';
}

/// @brief ページのすべての語が 0xFFFF(一度も書いていない)か.
bool mrd_eeprom_page_blank(const int16_t *a_w) {
  for (int k = 0; k < MRD_EEP_PAGE_WORDS; k++) {
    if (uint16_t(a_w[k]) != 0xFFFF) {
      return false;
    }
  }
  return true;
}

/// @brief ページに印・版・チェックサムを付ける(語[88]は0).
void mrd_eeprom_page_seal(int a_page, int16_t *a_w) {
  a_w[0] = int16_t(MRD_EEP_MARK + a_page);
  a_w[1] = MRD_EEP_VERSION;
  a_w[88] = 0;
  a_w[89] = mrd.cksm_val(a_w, MRD_EEP_PAGE_WORDS);
}

/// @brief ページが有効(印・版・チェックサムが合う)か.
bool mrd_eeprom_page_valid(int a_page, const int16_t *a_w) {
  return a_w[0] == int16_t(MRD_EEP_MARK + a_page) && a_w[1] == MRD_EEP_VERSION &&
         mrd.cksm_rslt(const_cast<int16_t *>(a_w), MRD_EEP_PAGE_WORDS);
}

/// @brief 旧形式(v1.1.1以前. 印なし)のサーボの設定のページか. P1 だけ, 空でなければ旧形式として読む.
bool mrd_eeprom_page_old_servo(int a_page, const int16_t *a_w) {
  return a_page == MRD_EEP_P_SERVO && !mrd_eeprom_page_valid(a_page, a_w) && !mrd_eeprom_page_blank(a_w);
}

/// @brief 設定値(cfg, config の配列)からページを作る.
/// @param a_page ページ番号.
/// @param a_w 作ったページ(90語).
void mrd_eeprom_page_make(int a_page, int16_t *a_w) {
  memset(a_w, 0, MRD_EEP_PAGE_WORDS * 2);
  for (int i = 0; i < MRD_EEP_FIELD_NUM; i++) {
    const MrdEepField &f_tmp = MRD_EEP_FIELDS[i];
    if (f_tmp.page != a_page) {
      continue;
    }
    int16_t *w_tmp = &a_w[f_tmp.word];
    switch (f_tmp.kind) {
    case MRD_EEP_INT:
    case MRD_EEP_U16:
      w_tmp[0] = int16_t(*static_cast<int *>(f_tmp.ptr));
      break;
    case MRD_EEP_U32: {
      const unsigned long v_tmp = *static_cast<unsigned long *>(f_tmp.ptr);
      w_tmp[0] = int16_t(v_tmp & 0xFFFF);
      w_tmp[1] = int16_t(v_tmp >> 16);
      break;
    }
    case MRD_EEP_FLOAT:
      w_tmp[0] = int16_t(constrain(lroundf(*static_cast<float *>(f_tmp.ptr) * f_tmp.scale), -32768L, 32767L)); // あふれない範囲に
      break;
    case MRD_EEP_INTS:
      for (int k = 0; k < f_tmp.scale; k++) {
        w_tmp[k] = int16_t(static_cast<int *>(f_tmp.ptr)[k]);
      }
      break;
    case MRD_EEP_STR:
      strncpy(reinterpret_cast<char *>(w_tmp), static_cast<char *>(f_tmp.ptr), f_tmp.scale);
      break;
    case MRD_EEP_BYTES_IP:
      mrd_eeprom_text_to_bytes(static_cast<char *>(f_tmp.ptr), '.', 10, reinterpret_cast<uint8_t *>(w_tmp), 4);
      break;
    case MRD_EEP_BYTES_MAC:
      mrd_eeprom_text_to_bytes(static_cast<char *>(f_tmp.ptr), ':', 16, reinterpret_cast<uint8_t *>(w_tmp), 6);
      break;
    }
  }
  if (a_page == MRD_EEP_P_SERVO || a_page == MRD_EEP_P_SLOT_EXT) { // スロットのページ(L/R の各スロットの2語)
    for (int i = 0; i < MRD_SERVO_SLOTS; i++) {
      for (int line = 0; line < 2; line++) {
        const int ix_tmp = ((line == 0) ? MRD_L_ORIGIDX : MRD_R_ORIGIDX) + i * 2;
        const int type_tmp = (line == 0) ? (i < IXL_MAX ? IXL_MT[i] : 0) : (i < IXR_MAX ? IXR_MT[i] : 0);
        const int id_tmp = (line == 0) ? (i < IXL_MAX ? IXL_ID[i] : 0) : (i < IXR_MAX ? IXR_ID[i] : 0);
        if (a_page == MRD_EEP_P_SERVO) { // 設定ワード(bit0 マウント, bit1-7 IDの下位7bit, bit8 回転方向)とトリム
          const int cw_tmp = (line == 0) ? (i < IXL_MAX ? IXL_CW[i] : 1) : (i < IXR_MAX ? IXR_CW[i] : 1);
          const float trim_tmp = (line == 0) ? (i < IXL_MAX ? IXL_TRIM[i] : 0) : (i < IXR_MAX ? IXR_TRIM[i] : 0);
          const int unmount_tmp = (line == 0) ? (i < IXL_MAX ? cfg.ixl_unmount[i] : 1) : (i < IXR_MAX ? cfg.ixr_unmount[i] : 1);
          const bool mount_tmp = (type_tmp != NOSERVO && !mrd_servo_is_pwm_type(type_tmp) && !unmount_tmp);
          a_w[ix_tmp] = int16_t((mount_tmp ? 0x0001 : 0) | ((id_tmp & 0x7F) << 1) | (cw_tmp > 0 ? 0x0100 : 0));
          a_w[ix_tmp + 1] = mrd.float2HfShort(trim_tmp);
        } else { // スロットの種類と8bitのID
          a_w[ix_tmp] = int16_t(type_tmp);
          a_w[ix_tmp + 1] = int16_t(id_tmp);
        }
      }
    }
  }
  mrd_eeprom_page_seal(a_page, a_w);
}

/// @brief ページの値を設定値(cfg, config の配列)に反映する. 範囲外の値は反映せず config の値のまま.
/// @param a_page ページ番号.
/// @param a_w ページ(90語).
/// @param a_p1_loaded P1(サーボのスロット)も EEPROM から読んだか. P2 のIDの決め方に使う.
/// @return 範囲外で反映しなかった項目の数.
int mrd_eeprom_page_apply(int a_page, const int16_t *a_w, bool a_p1_loaded) {
  int rejected_tmp = 0;
  // P5: SSID が空なら, SSID とパスワードは keys.h の値のまま
  const bool wifi_empty_tmp = (a_page == MRD_EEP_P_NET) && (a_w[MRD_EEP_NET_SSID] & 0xFF) == 0;
  for (int i = 0; i < MRD_EEP_FIELD_NUM; i++) {
    const MrdEepField &f_tmp = MRD_EEP_FIELDS[i];
    if (f_tmp.page != a_page) {
      continue;
    }
    if (wifi_empty_tmp && (f_tmp.ptr == cfg.wifi_ap_ssid || f_tmp.ptr == cfg.wifi_ap_pass)) {
      continue;
    }
    const int16_t *w_tmp = &a_w[f_tmp.word];
    long v_tmp = w_tmp[0];
    if (f_tmp.kind == MRD_EEP_U16) {
      v_tmp = uint16_t(w_tmp[0]);
    } else if (f_tmp.kind == MRD_EEP_U32) {
      v_tmp = long(uint16_t(w_tmp[0]) | (uint32_t(uint16_t(w_tmp[1])) << 16));
    }
    switch (f_tmp.kind) {
    case MRD_EEP_INT:
    case MRD_EEP_U16:
    case MRD_EEP_U32:
    case MRD_EEP_FLOAT:
      if (v_tmp < f_tmp.min || v_tmp > f_tmp.max) {
        rejected_tmp++;
        break;
      }
      if (f_tmp.kind == MRD_EEP_U32) {
        *static_cast<unsigned long *>(f_tmp.ptr) = (unsigned long)v_tmp;
      } else if (f_tmp.kind == MRD_EEP_FLOAT) {
        *static_cast<float *>(f_tmp.ptr) = float(v_tmp) / f_tmp.scale;
      } else {
        *static_cast<int *>(f_tmp.ptr) = int(v_tmp);
      }
      break;
    case MRD_EEP_INTS:
      for (int k = 0; k < f_tmp.scale; k++) {
        if (w_tmp[k] < f_tmp.min || w_tmp[k] > f_tmp.max) {
          rejected_tmp++;
        } else {
          static_cast<int *>(f_tmp.ptr)[k] = w_tmp[k];
        }
      }
      break;
    case MRD_EEP_STR:
      memcpy(f_tmp.ptr, w_tmp, f_tmp.scale);
      static_cast<char *>(f_tmp.ptr)[f_tmp.scale] = '\0';
      break;
    case MRD_EEP_BYTES_IP:
    case MRD_EEP_BYTES_MAC:
      mrd_eeprom_bytes_to_text(reinterpret_cast<const uint8_t *>(w_tmp), f_tmp.kind == MRD_EEP_BYTES_IP ? 4 : 6,
                               f_tmp.kind == MRD_EEP_BYTES_IP ? '.' : ':', f_tmp.kind == MRD_EEP_BYTES_MAC,
                               static_cast<char *>(f_tmp.ptr));
      break;
    }
  }
  if (a_page == MRD_EEP_P_SERVO || a_page == MRD_EEP_P_SLOT_EXT) {
    for (int i = 0; i < MRD_SERVO_SLOTS; i++) {
      for (int line = 0; line < 2; line++) {
        if (i >= ((line == 0) ? IXL_MAX : IXR_MAX)) {
          continue;
        }
        const int ix_tmp = ((line == 0) ? MRD_L_ORIGIDX : MRD_R_ORIGIDX) + i * 2;
        int *id_tmp = (line == 0) ? &IXL_ID[i] : &IXR_ID[i];
        if (a_page == MRD_EEP_P_SERVO) {
          const uint16_t word_tmp = uint16_t(a_w[ix_tmp]);
          ((line == 0) ? cfg.ixl_unmount : cfg.ixr_unmount)[i] = (word_tmp & 0x0001) ? 0 : 1;
          *id_tmp = (word_tmp >> 1) & 0x7F;
          ((line == 0) ? IXL_CW : IXR_CW)[i] = (word_tmp & 0x0100) ? 1 : -1;
          ((line == 0) ? IXL_TRIM : IXR_TRIM)[i] = a_w[ix_tmp + 1] / 100.0f;
        } else {
          const int type_tmp = a_w[ix_tmp];
          const int id8_tmp = a_w[ix_tmp + 1];
          if (type_tmp < 0 || type_tmp > 127 || id8_tmp < 0 || id8_tmp > 253) {
            rejected_tmp++;
            continue;
          }
          ((line == 0) ? IXL_MT : IXR_MT)[i] = type_tmp;
          // ID: P1 も EEPROM から読んだときは, P2 のIDの下位7bitが P1 のIDと同じ場合だけ P2 のID(128以上を含む)を使う.
          //     違えば Meridian_console で P1 のIDを変えたとみなし, P1 のID(0〜127)のまま.
          if (!a_p1_loaded || (id8_tmp & 0x7F) == *id_tmp) {
            *id_tmp = id8_tmp;
          }
        }
      }
    }
  }
  return rejected_tmp;
}

//==================================================================================================
//  EEPROMの読み書き
//==================================================================================================

// EEPROMの内容の作業用. 2880byte あるため, 関数の中(スタック)に置かず, ここに1つだけ置く
EepromUnion mrd_eep_buf;

/// @brief EEPROMの内容を作業用の配列(mrd_eep_buf)に読み込む.
/// @return 読み込んだ配列(mrd_eep_buf. 次の読み込みで上書きされる).
EepromUnion &mrd_eeprom_read() {
  for (int i = 0; i < EEPROM_SIZE; i++) {
    mrd_eep_buf.bval[i] = EEPROM.read(i);
  }
  return mrd_eep_buf;
}

/// @brief EEPROMが空(一度も書き込んでいない. 全バイトが0xFF)か.
/// @param a_data EEPROMの内容.
/// @return 空ならtrue.
bool mrd_eeprom_is_blank(const EepromUnion &a_data) {
  for (int i = 0; i < EEPROM_SIZE; i++) {
    if (a_data.bval[i] != 0xFF) {
      return false;
    }
  }
  return true;
}

/// @brief EEPROMにEEPROM格納用の配列データを書き込む(内容が違うbyteだけ書き, まとめて確定する).
/// @param a_write_data EEPROM書き込み用の配列データ.
/// @param a_flg_protect 書き込み禁止(EEPROM_PROTECT)か.
/// @return EEPROMの内容が書き込みデータと一致した場合はtrueを, プロテクト等で書き込めなかった場合はfalseを返す.
bool mrd_eeprom_write(const EepromUnion &a_write_data, bool a_flg_protect, HardwareSerial &a_serial) {
  if (a_flg_protect) { // 書き込み禁止(EEPROM_PROTECT)をチェック
    a_serial.println("EEPROM is protected. To unprotect, please set 'EEPROM_PROTECT' to false.");
    return false;
  }
  if (EEPROM.length() < EEPROM_SIZE) {
    a_serial.println("Error: EEPROM address out of range.");
    return false;
  }
  bool flg_renew_tmp = false; // 書き込みコミットを実施するかのフラグ
  for (int i = 0; i < EEPROM_SIZE; i++) {
    if (EEPROM.read(i) != a_write_data.bval[i]) { // 書き込みデータがEEPROM内のデータと違う場合のみ書く
      EEPROM.write(i, a_write_data.bval[i]);
      flg_renew_tmp = true;
    }
  }
  if (flg_renew_tmp && !EEPROM.commit()) { // 書き込みを確定する
    a_serial.println("Error: EEPROM commit failed.");
    return false;
  }
  return true;
}

//------------------------------------------------------------------------------------
//  起動時の処理
//------------------------------------------------------------------------------------

// 起動時の処理の結果(シリアルの開始後に表示するため保持する)
struct MrdEepBoot {
  bool began = false;     // EEPROMを開始できたか
  uint16_t loaded = 0;    // EEPROM の値を使ったページ(ビット)
  uint16_t invalid = 0;   // EEPROM を選んだが空・壊れていて config の値を使ったページ(ビット)
  uint16_t old_servo = 0; // P1 を旧形式として読んだか(ビット)
  uint16_t written = 0;   // config の値で書いたページ(ビット)
  int rejected = 0;       // 範囲外で反映しなかった項目の数
  bool write_failed = false;
};
MrdEepBoot mrd_eep_boot;

/// @brief 起動の最初に, EEPROM を開始して設定値を決める(シリアルの開始より前. 表示はしない).
///        EEPROM_LOAD で選んだページは, 有効なら EEPROM の値を使う. それ以外は config の値で,
///        EEPROM_SET 1 なら config の値でそのページを書く.
/// @param a_load_mask EEPROM の値を使うページ(EEPROM_LOAD).
/// @param a_set config の値を書くか(EEPROM_SET).
/// @param a_protect 書き込み禁止か(EEPROM_PROTECT).
void mrd_eeprom_settings_boot(int a_load_mask, bool a_set, bool a_protect) {
  MrdEepBoot &b_tmp = mrd_eep_boot;
  b_tmp.began = EEPROM.begin(EEPROM_SIZE);
  if (!b_tmp.began) {
    return; // EEPROMを使えない: すべて config の値
  }
  EepromUnion &e_tmp = mrd_eeprom_read();
  bool use_tmp[MRD_EEP_P_USED];
  for (int i = 0; i < MRD_EEP_P_USED; i++) {
    const bool valid_tmp = mrd_eeprom_page_valid(i, e_tmp.saval[i]);
    const bool old_tmp = mrd_eeprom_page_old_servo(i, e_tmp.saval[i]);
    const bool selected_tmp = (a_load_mask >> i) & 1;
    use_tmp[i] = selected_tmp && (valid_tmp || old_tmp);
    if (selected_tmp && !use_tmp[i]) {
      b_tmp.invalid |= (1 << i);
    }
    if (use_tmp[i] && old_tmp) {
      b_tmp.old_servo |= (1 << i);
    }
  }
  // config の値を使うページを, config の値で書く(EEPROM の値を使うページは書かない)
  if (a_set && !a_protect) {
    int16_t page_tmp[MRD_EEP_PAGE_WORDS];
    for (int i = 0; i < MRD_EEP_P_USED; i++) {
      if (!use_tmp[i]) {
        mrd_eeprom_page_make(i, page_tmp);
        if (memcmp(page_tmp, e_tmp.saval[i], sizeof(page_tmp)) != 0) { // 内容が違うページだけ書く
          EEPROM.writeBytes(i * sizeof(page_tmp), page_tmp, sizeof(page_tmp));
          b_tmp.written |= (1 << i);
        }
      }
    }
    if (b_tmp.written) {
      b_tmp.write_failed = !EEPROM.commit();
    }
  }
  // EEPROM の値を使うページを反映する(P1 → P2 の順. P2 のIDは P1 を読んだかで決まる)
  for (int i = 0; i < MRD_EEP_P_USED; i++) {
    if (use_tmp[i]) {
      b_tmp.rejected += mrd_eeprom_page_apply(i, e_tmp.saval[i], use_tmp[MRD_EEP_P_SERVO]);
      b_tmp.loaded |= (1 << i);
    }
  }
}

/// @brief 起動時の処理の結果を表示する.
void mrd_eeprom_boot_report(HardwareSerial &a_serial) {
  const MrdEepBoot &b_tmp = mrd_eep_boot;
  a_serial.print("Initializing EEPROM... ");
  if (!b_tmp.began) {
    a_serial.println("Failed (all settings from config)");
    return;
  }
  a_serial.print("OK (");
  a_serial.print(EEPROM_SIZE);
  a_serial.println(" bytes)");
  const char *names_tmp[MRD_EEP_P_USED] = {"system", "servo", "slot", "line", "pwm", "net"};
  a_serial.print("Settings: ");
  for (int i = 0; i < MRD_EEP_P_USED; i++) {
    a_serial.print("P");
    a_serial.print(i);
    a_serial.print(" ");
    a_serial.print(names_tmp[i]);
    a_serial.print((b_tmp.loaded >> i) & 1 ? "=EEPROM" : "=config");
    if ((b_tmp.old_servo >> i) & 1) {
      a_serial.print("(old format)");
    }
    a_serial.print(i < MRD_EEP_P_USED - 1 ? ", " : "\n");
  }
  if (b_tmp.invalid) {
    a_serial.print("EEPROM pages empty or broken (config settings are used): ");
    for (int i = 0; i < MRD_EEP_P_USED; i++) {
      if ((b_tmp.invalid >> i) & 1) {
        a_serial.print("P");
        a_serial.print(i);
        a_serial.print(" ");
      }
    }
    a_serial.println();
  }
  if (b_tmp.rejected) {
    a_serial.print("EEPROM values out of range (config settings are used): ");
    a_serial.println(b_tmp.rejected);
  }
  if (b_tmp.written) {
    a_serial.println(b_tmp.write_failed ? "Write EEPROM failed." : "Set EEPROM data from config.");
  }
}

//------------------------------------------------------------------------------------
//  Meridian 通信での読み書き(10200+p, 10300+p, 10101)
//------------------------------------------------------------------------------------

/// @brief EEPROM のページを Meridim の [2]〜[87] に入れる.
/// @param a_page ページ番号(0〜MRD_EEP_PAGES-1).
/// @param a_meridim 送信用のMeridim配列. [0][1][88][89] は変えない.
void mrd_eeprom_page_to_meridim(int a_page, Meridim90Union &a_meridim) {
  const EepromUnion &e_tmp = mrd_eeprom_read();
  for (int k = MRD_EEP_DATA_FIRST; k <= MRD_EEP_DATA_LAST; k++) {
    a_meridim.sval[k] = e_tmp.saval[a_page][k];
  }
}

/// @brief Meridim の [2]〜[87] を EEPROM のページに書く(印・版・チェックサムはボードが付ける).
/// @param a_page ページ番号(0〜MRD_EEP_PAGES-1).
/// @param a_meridim 受信したMeridim配列.
/// @param a_flg_protect 書き込み禁止(EEPROM_PROTECT)か.
/// @return 書けたらtrue.
bool mrd_eeprom_page_store(int a_page, const Meridim90Union &a_meridim, bool a_flg_protect, HardwareSerial &a_serial) {
  EepromUnion &e_tmp = mrd_eeprom_read();
  int16_t *w_tmp = e_tmp.saval[a_page];
  memset(w_tmp, 0, MRD_EEP_PAGE_WORDS * 2);
  for (int k = MRD_EEP_DATA_FIRST; k <= MRD_EEP_DATA_LAST; k++) {
    w_tmp[k] = a_meridim.sval[k];
  }
  mrd_eeprom_page_seal(a_page, w_tmp);
  return mrd_eeprom_write(e_tmp, a_flg_protect, a_serial);
}

/// @brief EEPROM の P1(サーボのスロット)のトリムと回転方向をサーボの設定に反映する(10100, 10102, 10300+1).
///        マウント・IDは次の起動から反映する(起動時の初期化と食い違わないよう, 動作中は変えない).
///        EEPROMが空, または P1 が無効(旧形式を除く)なら表示だけして読み込まない.
/// @param a_sv サーボ設定を保持する構造体.
/// @param a_monitor シリアルモニタへのデータ表示.
/// @param a_serial 出力先シリアルの指定.
/// @return 読み込んだらtrue.
bool mrd_eeprom_servosettings_load(ServoParam &a_sv, bool a_monitor, HardwareSerial &a_serial) {
  const EepromUnion &e_tmp = mrd_eeprom_read();
  if (mrd_eeprom_is_blank(e_tmp)) {
    a_serial.println("EEPROM is empty. Servo settings are not loaded from EEPROM (config settings are used).");
    return false;
  }
  const int16_t *w_tmp = e_tmp.saval[MRD_EEP_P_SERVO];
  if (!mrd_eeprom_page_valid(MRD_EEP_P_SERVO, w_tmp) && !mrd_eeprom_page_old_servo(MRD_EEP_P_SERVO, w_tmp)) {
    a_serial.println("EEPROM servo settings (P1) are empty or broken. Not loaded.");
    return false;
  }
  a_serial.println("Load trim and direction from EEPROM.");
  for (int i = 0; i < a_sv.num_max; i++) {
    a_sv.ixl_cw[i] = (uint16_t(w_tmp[MRD_L_ORIGIDX + i * 2]) & 0x0100) ? 1 : -1; // bit8:回転方向
    a_sv.ixr_cw[i] = (uint16_t(w_tmp[MRD_R_ORIGIDX + i * 2]) & 0x0100) ? 1 : -1;
    a_sv.ixl_trim[i] = w_tmp[MRD_L_ORIGIDX + 1 + i * 2] / 100.0f; // トリム(degree ×100)
    a_sv.ixr_trim[i] = w_tmp[MRD_R_ORIGIDX + 1 + i * 2] / 100.0f;
    if (a_monitor) {
      a_serial.print("L-idx:");
      a_serial.print(mrd_pddstr(i, 2, 0, false));
      a_serial.print(", cw:");
      a_serial.print(mrd_pddstr(a_sv.ixl_cw[i], 1, 0, true));
      a_serial.print(", trm:");
      a_serial.print(mrd_pddstr(a_sv.ixl_trim[i], 7, 2, true));
      a_serial.print("  R-idx: ");
      a_serial.print(mrd_pddstr(i, 2, 0, false));
      a_serial.print(", cw:");
      a_serial.print(mrd_pddstr(a_sv.ixr_cw[i], 1, 0, true));
      a_serial.print(", trm:");
      a_serial.println(mrd_pddstr(a_sv.ixr_trim[i], 7, 2, true));
    }
  }
  return true;
}

//------------------------------------------------------------------------------------
//  表示
//------------------------------------------------------------------------------------

/// @brief 使っているページの内容をシリアルに表示する.
/// @param a_data EEPROM用の配列データ.
/// @param a_bhd ダンプリストの表示形式.(0:Bin, 1:Hex, 2:Dec)
/// @return 終了時にtrueを返す.
bool mrd_eeprom_dump_to_serial(const EepromUnion &a_data, int a_bhd, HardwareSerial &a_serial) {
  a_serial.print("EEPROM Length ");
  a_serial.print(EEPROM.length());
  a_serial.println("byte, 16bit Dump (P0-P5);");
  for (int i = 0; i < MRD_EEP_P_USED; i++) {
    a_serial.print("P");
    a_serial.print(i);
    a_serial.print(": ");
    for (int k = 0; k < MRD_EEP_PAGE_WORDS; k++) {
      const int16_t v_tmp = a_data.saval[i][k];
      if (a_bhd == 0) {
        a_serial.print(v_tmp, BIN);
      } else if (a_bhd == 1) {
        a_serial.print(v_tmp, HEX);
      } else {
        a_serial.print(v_tmp, DEC);
      }
      a_serial.print(k < MRD_EEP_PAGE_WORDS - 1 ? "/" : "\n");
    }
  }
  return true;
}

/// @brief EEPROM格納用の配列データをシリアルにダンプ出力する.(起動時用)
/// @param a_do_dump 実施するか否か.
/// @param a_bhd ダンプリストの表示形式.(0:Bin, 1:Hex, 2:Dec)
/// @return 終了時にtrueを返す.
bool mrd_eeprom_dump_at_boot(bool a_do_dump, int a_bhd, HardwareSerial &a_serial) {
  if (a_do_dump) {
    mrd_eeprom_dump_to_serial(mrd_eeprom_read(), a_bhd, a_serial);
    return true;
  }
  return false;
}

#endif // __MERIDIAN_EEPROM_H__
