#ifndef __MERIDIAN_SERVO_JR_XBUS_H__
#define __MERIDIAN_SERVO_JR_XBUS_H__

#include "config.h"
#include "main.h"
#include "mrd_servo.h" // 半二重UARTバスと共通処理. 本ファイルは mrd_servo.h から読み込まれる
#include "sv_util.h"   // 各プロトコル層の共通部品

//==================================================================================================
//  JR PROPO XBUS プロトコル層 (半二重UARTバス上で動作)
//==================================================================================================
//
// 対象: XBUSサーボ(NX8921等). 参照: JR「XBus サーボ プロトコル仕様概略 Ver.1.0.6」, JR_XBUS_Servo_Setter.
//
// 通信: 250000bps 8N1(固定). 待機中は送信方向(線をESP32側が駆動する. 参照実装と同じ).
// 1フレームの処理:
//   全サーボの目標位置を1つのチャンネルデータ(0xA4)で送る. 返信はない(動作命令のみ).
//   Meridianの100Hzのうち偶数フレームだけ送り, 50Hzで更新する. 15台で65byte=約2.6ms.
//   トルクOFF(コマンド1以外)のサーボには位置0を送って脱力させる(XBUSにはトルクON/OFFの命令がない).
//   ※チャンネルデータが約1.5秒途絶えるとサーボは脱力する(StopMode設定による).
//   ※仕様の送信周期は14msだが, 本実装は20ms(50Hz)で送る(実機未確認).
// 戻り値: 現在位置は読まず, 指令値をそのまま返す(tgtを書き換えない).
//
// 位置値: 0x1249/0x7FFF/0xEDB6 が 900/1500/2100us. 通常モードは±60deg(0.10deg/us),
//         180度モード(サーボ側の設定)は±90deg(0.15deg/us). SERVO_XBUS_ANGLE180 で合わせる.

#define XBUS_CMD_CHANNEL_DATA 0xA4   // チャンネルデータ(全サーボの目標位置)
#define XBUS_POS_NEUTRAL      0x7FFF // 1500us
#define XBUS_POS_MIN          0x1249 // 900us
#define XBUS_POS_MAX          0xEDB6 // 2100us
#define XBUS_POS_RELEASE      0x0000 // 脱力
#define XBUS_PULSE_HALF_US    600.0f // 中央から900us/2100usまでの幅
#define XBUS_ID_MIN           1
#define XBUS_ID_MAX           50
#define XBUS_TX_MAX           MRD_HDUART_TX_MAX // 1パケットの最大長

/// @brief CRC-8(反転多項式0x8C, 初期値0). 先頭からCRCの手前まで.
inline uint8_t mrd_xbus_crc8(const uint8_t *a_data, size_t a_len) {
  uint8_t crc_tmp = 0;
  for (size_t i = 0; i < a_len; i++) {
    crc_tmp ^= a_data[i];
    for (int b = 0; b < 8; b++) {
      crc_tmp = (crc_tmp & 0x01) ? uint8_t((crc_tmp >> 1) ^ 0x8C) : uint8_t(crc_tmp >> 1);
    }
  }
  return crc_tmp;
}

/// @brief 角度(degree)をXBUSの位置値に変換する(900-2100usの範囲に制限).
/// @param a_degree 角度(degree).
/// @param a_trim トリム値(degree).
/// @param a_cw 回転方向補正値(+1 or -1).
inline uint16_t mrd_xbus_deg2pos(float a_degree, float a_trim, int a_cw) {
  const float deg_per_us_tmp = cfg.servo_xbus_angle180 ? 0.15f : 0.10f;
  float us_tmp = (a_trim + a_degree * a_cw) / deg_per_us_tmp;
  us_tmp = constrain(us_tmp, -XBUS_PULSE_HALF_US, XBUS_PULSE_HALF_US);
  const float span_tmp = (us_tmp >= 0) ? float(XBUS_POS_MAX - XBUS_POS_NEUTRAL) : float(XBUS_POS_NEUTRAL - XBUS_POS_MIN);
  const long pos_tmp = XBUS_POS_NEUTRAL + lroundf(us_tmp / XBUS_PULSE_HALF_US * span_tmp);
  return uint16_t(constrain(pos_tmp, long(XBUS_POS_MIN), long(XBUS_POS_MAX)));
}

//------------------------------------------------------------------------------------
//  XBUSサーボ1系統分のクラス
//------------------------------------------------------------------------------------

class XbusHalfDuplexClass {
public:
  /// @brief コンストラクタ.
  /// @param a_bus 使用する半二重UARTバス.
  /// @param a_baud 通信速度bps(XBUSは250000固定).
  XbusHalfDuplexClass(MrdHalfDuplexUart &a_bus, unsigned long a_baud) : m_bus(a_bus), m_baud(a_baud) {}

  /// @brief 通信速度を設定し直す(起動時に EEPROM の設定を反映するため). begin() の前に呼ぶ.
  void set_baud(unsigned long a_baud) { m_baud = a_baud; }

  /// @brief バスを開始(8N1, 送信方向で待機)し, 設定を表示する. 返信を使わないためサーボの確認はしない.
  /// @param a_line_name 表示用の系統名("L"など).
  /// @param a_mount 各サーボのマウント有無.
  /// @param a_id 各サーボのID.
  /// @param a_num サーボ数.
  /// @return バスを開始できればtrue.
  bool begin(const char *a_line_name, const int *a_mount, const int *a_id, int a_num) {
    if (!m_bus.begin(m_baud, SERIAL_8N1, false, true)) {
      return false;
    }
    const unsigned long bauds_tmp[] = {250000};
    mrd_servo_baud_check(a_line_name, "XBUS", m_baud, bauds_tmp, 1, " bps is not the XBUS baud rate. Set SERVO_BAUDRATE_x to 250000.");
    for (int i = 0; i < a_num && i < MRD_SERVO_SLOTS; i++) {
      if (a_mount[i] && (a_id[i] < XBUS_ID_MIN || a_id[i] > XBUS_ID_MAX)) {
        Serial.print("UART_");
        Serial.print(a_line_name);
        Serial.print(" XBUS [");
        Serial.print(i);
        Serial.print("] ID ");
        Serial.print(a_id[i]);
        Serial.println(" is out of range (1-50), ignored.");
      }
    }
    return true;
  }

  /// @brief 半二重UARTバス(並行送受信用).
  MrdHalfDuplexUart &bus() { return m_bus; }

  /// @brief フレームをまたいで保持する状態(本プロトコルでは使わないが, 系統の参照の形を揃えるため持つ).
  MrdServoLineState m_state;

private:
  MrdHalfDuplexUart &m_bus;
  unsigned long m_baud;
};

//==================================================================================================
//  XBUSサーボの駆動処理
//==================================================================================================

/// @brief 系統のXBUSサーボクラス.
inline XbusHalfDuplexClass &mrd_xbus_line(int a_line) { return (a_line == L) ? xbus_l : xbus_r; }

/// @brief 系統の参照を作る.
MrdServoLineRef mrd_xbus_line_ref(UartLine a_line, Meridim90Union &a_meridim, ServoParam &a_sv) {
  return mrd_servo_line_ref(a_line, JRXBUS, mrd_xbus_line(a_line).bus(), mrd_xbus_line(a_line).m_state, a_meridim, a_sv);
}

/// @brief XBUSサーボ1系統分の1フレームの処理. 偶数フレームだけ全サーボのチャンネルデータを1回送る.
class XbusLineStepper : public MrdLineStepper {
public:
  /// @param a_line 系統の参照.
  /// @param a_num 処理するサーボ数.
  XbusLineStepper(const MrdServoLineRef &a_line, int a_num) : m_ln(a_line), m_num(min(a_num, MRD_SERVO_SLOTS)) {}

  bool next(MrdHduartXfer &a_x) override {
    if (m_done || (mrd_servo_frame % 2) != 0) {
      return false;
    }
    m_done = true;
    const size_t len_tmp = build_channel_data();
    if (len_tmp == 0) {
      return false;
    }
    a_x.bus = m_ln.bus;
    a_x.tx = m_buf;
    a_x.tx_len = len_tmp;
    a_x.rx_len = 0;
    return true;
  }

  void complete(const MrdHduartXfer &) override {} // 返信なし. 指令値をそのまま返す

private:
  /// @brief チャンネルデータを作る: A4 | LEN | 00(key) | 00(type) | {ID, 00, 位置H, 位置L} x N | CRC
  /// @return パケット長(対象のサーボがなければ0).
  size_t build_channel_data() {
    size_t pos_tmp = 4;
    for (int i = 0; i < m_num; i++) {
      if (!m_ln.mount[i] || m_ln.id[i] < XBUS_ID_MIN || m_ln.id[i] > XBUS_ID_MAX || pos_tmp + 4 + 1 > sizeof(m_buf)) {
        continue;
      }
      const uint16_t val_tmp = m_ln.on(i) ? mrd_xbus_deg2pos(m_ln.tgt[i], m_ln.trim[i], m_ln.cw[i]) : XBUS_POS_RELEASE;
      m_buf[pos_tmp++] = m_ln.id[i];
      m_buf[pos_tmp++] = 0x00;
      m_buf[pos_tmp++] = val_tmp >> 8;
      m_buf[pos_tmp++] = val_tmp & 0xFF;
    }
    if (pos_tmp == 4) {
      return 0;
    }
    m_buf[0] = XBUS_CMD_CHANNEL_DATA;
    m_buf[1] = pos_tmp - 2; // key + type + データ
    m_buf[2] = 0x00;
    m_buf[3] = 0x00;
    m_buf[pos_tmp] = mrd_xbus_crc8(m_buf, pos_tmp);
    return pos_tmp + 1;
  }

  MrdServoLineRef m_ln;
  int m_num;
  bool m_done = false;
  uint8_t m_buf[XBUS_TX_MAX];
};

#endif // __MERIDIAN_SERVO_JR_XBUS_H__
