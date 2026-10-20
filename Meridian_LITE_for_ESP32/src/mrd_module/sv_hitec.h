#ifndef __MERIDIAN_SERVO_HITEC_HSR_H__
#define __MERIDIAN_SERVO_HITEC_HSR_H__

#include "config.h"
#include "main.h"
#include "mrd_servo.h" // 半二重UARTバスと共通処理. 本ファイルは mrd_servo.h から読み込まれる
#include "sv_util.h"   // 各プロトコル層の共通部品

//==================================================================================================
//  Hitec HSR (HMIシリアル) プロトコル層 (半二重UARTバス上で動作)
//==================================================================================================
//
// 対象: HSR-8498HB (HMIシリアル). 参照: Hitec_HSR_Servo_Setter (非公式の解析資料に基づく).
// 系統内は全台この方式とし, 他のサーボと同じ系統に混在させない(通信速度と信号の極性が違うため).
// 宛先なしの読み出しは返信が衝突するため, 起動時のサーボの自動判定はせず, configの指定に従う.
//
// 通信: 19200bps 8N2, 信号反転(アイドルLOW). 待機中は送信方向(線をESP32側が駆動する. 参照実装と同じ).
// パケット: 0x80 | CMD | DATA1 | DATA2 | SUM の5バイトを送り, 続く2バイトの枠にサーボが返信を書き込む.
//           SUM = 0x100 - (先頭4バイトの和).
// 通信速度が遅いため, 動作中と脱力中で処理を分ける. どちらも1台ずつで, 1台あたり約4.2ms(送信5byte+返信枠2byte).
//   動作中(トルクONのサーボがある): 位置命令(CMD=ID)だけを送り, 返信(00 00)は待たない(返信枠の分だけ線を開放する).
//     1フレームに送る台数を SERVO_HTC_PER_FRAME 台までとし, トルクONのサーボに順番に送る.
//     各サーボは連続するフレームでは送らず, 50Hzを超えない. 台数が多いと各サーボの更新は50Hzより遅くなる
//     (1台/フレームなら, 2台までは各50Hz, 4台なら各25Hz).
//     戻り値: 現在位置は読まず, 指令値をそのまま返す(tgtを書き換えない).
//   脱力中(全サーボがトルクOFF): 角度値は送らず, 現在位置の読み出し(0xE9)だけを行う.
//     1フレームに SERVO_HTC_PER_FRAME 台ずつ順番に読み, 読み取った角度を返す. 読まないサーボは前回の値.
//     0xE9は速度の設定を兼ねるため, スロットごとの速度(初期値 SERVO_HTC_SPEED, パラメータ系コマンド53で変更)を送る.
//     返信はホストの区切りで受ける(ESP32が各バイトのスタート/ストップビットを出し, データビットの間だけ線を開放する).
//   脱力: 個別に脱力する命令はなく, 0xEFで全台が脱力する. 位置命令を受けると全台のトルクが戻る.
//         そのため全サーボがトルクOFFになったときと, トルク状態の送り直し間隔ごとに0xEFを送る(その回は読み出しの代わり).
//         一部だけOFFの場合, そのサーボには送らず, 最後の位置を保持したままになる(指令値をそのまま返す).
//
// 位置: パルス幅(us). 1500usが中央, 1deg = SERVO_HTC_US_PER_DEG us, 550〜2450usに制限.

#define HTC_HEADER          0x80 // 先頭バイト(サーボをシリアル受信状態にする)
#define HTC_CMD_FREE        0xEF // 全サーボ脱力(ID指定なし)
#define HTC_CMD_SPEED_POS   0xE9 // 速度の設定と位置の読み出し(DATA1=ID, DATA2=速度. 返信=位置H, 位置L)
#define HTC_ID_MAX          0x7F
#define HTC_PULSE_C         1500.0f
#define HTC_PULSE_MIN       550.0f
#define HTC_PULSE_MAX       2450.0f
#define HTC_REPLY_LEN       2   // 返信のバイト数
#define HTC_REPLY_BITS      22  // 返信2バイト分のビット数(8N2: 11bit x 2)
#define HTC_READ_MIN        300 // 読み取った位置(us)として受け付ける範囲(範囲外は通信エラー扱い)
#define HTC_READ_MAX        2700
#define HTC_REPLY_MARGIN_US 200 // 返信枠の待ちの余裕(us)

/// @brief 5バイトの命令パケットを作る.
/// @return パケット長(5).
inline size_t mrd_htc_packet_make(uint8_t *a_buf, uint8_t a_cmd, uint8_t a_data1, uint8_t a_data2) {
  a_buf[0] = HTC_HEADER;
  a_buf[1] = a_cmd;
  a_buf[2] = a_data1;
  a_buf[3] = a_data2;
  a_buf[4] = uint8_t(0x100 - uint8_t(a_buf[0] + a_buf[1] + a_buf[2] + a_buf[3]));
  return 5;
}

/// @brief 角度(degree)をHitecのパルス幅(us)に変換する(550-2450usに制限).
inline uint16_t mrd_htc_deg2us(float a_degree, float a_trim, int a_cw) {
  return mrd_servo_deg2pos(a_degree, a_trim, a_cw, cfg.servo_htc_us_per_deg, HTC_PULSE_C, HTC_PULSE_MIN, HTC_PULSE_MAX);
}

/// @brief Hitecのパルス幅(us)を角度(degree)に変換する. mrd_htc_deg2us の逆変換.
inline float mrd_htc_us2deg(int a_us, float a_trim, int a_cw) {
  return mrd_servo_pos2deg(a_us, a_trim, a_cw, cfg.servo_htc_us_per_deg, HTC_PULSE_C);
}

//------------------------------------------------------------------------------------
//  Hitecサーボ1系統分のクラス
//------------------------------------------------------------------------------------

class HitecHalfDuplexClass {
public:
  /// @brief コンストラクタ.
  /// @param a_bus 使用する半二重UARTバス.
  /// @param a_baud 通信速度bps(HMIは19200固定).
  HitecHalfDuplexClass(MrdHalfDuplexUart &a_bus, unsigned long a_baud) : m_bus(a_bus), m_baud(a_baud) {}

  /// @brief 通信速度を設定し直す(起動時に EEPROM の設定を反映するため). begin() の前に呼ぶ.
  void set_baud(unsigned long a_baud) { m_baud = a_baud; }

  /// @brief バスを開始(8N2, 信号反転, 送信方向で待機)し, 設定を表示する.
  ///        読み出しの宛先なし命令は複数台で返信が衝突するため, 起動時のサーボの確認はしない.
  /// @param a_line_name 表示用の系統名("L"など).
  /// @param a_mount 各サーボのマウント有無.
  /// @param a_id 各サーボのID.
  /// @param a_num サーボ数.
  /// @return バスを開始できればtrue.
  bool begin(const char *a_line_name, const int *a_mount, const int *a_id, int a_num) {
    m_state.reset();
    for (int i = 0; i < MRD_SERVO_SLOTS; i++) {
      m_speed[i] = cfg.servo_htc_speed;
    }
    m_freed = false;
    m_sent_prev = 0;
    m_sent_cur = 0;
    if (!m_bus.begin(m_baud, SERIAL_8N2, true, true)) {
      return false;
    }
    const unsigned long bauds_tmp[] = {19200};
    mrd_servo_baud_check(a_line_name, "HITEC", m_baud, bauds_tmp, 1, " bps is not the HMI baud rate. Set SERVO_BAUDRATE_x to 19200.");
    int num_tmp = 0;
    for (int i = 0; i < a_num && i < MRD_SERVO_SLOTS; i++) {
      if (!a_mount[i]) {
        continue;
      }
      num_tmp++;
      if (a_id[i] < 0 || a_id[i] > HTC_ID_MAX) {
        Serial.print("UART_");
        Serial.print(a_line_name);
        Serial.print(" HITEC [");
        Serial.print(i);
        Serial.print("] ID ");
        Serial.print(a_id[i]);
        Serial.println(" is out of range (0-127), ignored.");
      }
    }
    if (num_tmp > 2 * cfg.servo_htc_per_frame) {
      Serial.print("UART_");
      Serial.print(a_line_name);
      Serial.print(" HITEC: ");
      Serial.print(num_tmp);
      Serial.print(" servos, each updated at about ");
      Serial.print(100 * cfg.servo_htc_per_frame / num_tmp);
      Serial.println(" Hz (sent / read one by one).");
    }
    return true;
  }

  /// @brief 返信枠の待ち時間(us).
  unsigned long reply_hold_us() const { return HTC_REPLY_BITS * 1000000UL / max(m_baud, 1200UL) + HTC_REPLY_MARGIN_US; }

  /// @brief 半二重UARTバス(並行送受信用).
  MrdHalfDuplexUart &bus() { return m_bus; }

  /// @brief フレームをまたいで保持する状態(順番に送るサーボの位置, 脱力の送り直しの間隔).
  MrdServoLineState m_state;
  uint8_t m_speed[MRD_SERVO_SLOTS]; // スロットごとの速度(1-255. 0xE9で送る. RAMのみ)
  uint16_t m_sent_prev = 0;         // 前のフレームで位置を送ったサーボ(bit i: インデックスi)
  uint16_t m_sent_cur = 0;          // このフレームで位置を送ったサーボ
  bool m_freed = false;             // 全台脱力(0xEF)を送ってから位置命令を送っていないか

private:
  MrdHalfDuplexUart &m_bus;
  unsigned long m_baud;
};

//==================================================================================================
//  Hitecサーボの駆動処理
//==================================================================================================

/// @brief 系統のHitecサーボクラス.
inline HitecHalfDuplexClass &mrd_htc_line(int a_line) { return (a_line == L) ? htc_l : htc_r; }

/// @brief 系統の参照を作る.
MrdServoLineRef mrd_htc_line_ref(UartLine a_line, Meridim90Union &a_meridim, ServoParam &a_sv) {
  return mrd_servo_line_ref(a_line, HTCHSR, mrd_htc_line(a_line).bus(), mrd_htc_line(a_line).m_state, a_meridim, a_sv);
}

//------------------------------------------------------------------------------------
//  パラメータ系コマンド (番号と値の形式は tasks/servo_param_plan.md)
//------------------------------------------------------------------------------------
//
// HMIでIDを指定して送れる読み書きは 0xE9(速度の設定と位置の読み出し)だけで, EEPROMの読み書き(0xE1/0xE2)は
// IDを持たず系統の全サーボに届くため, 1系統に複数台をつなぐMeridianでは使わない. 対応するのは
// 3: 応答確認, 6: 現在角度, 53: スピード(1-255, RAMのみ. 以後の0xE9で送る) のみ.

/// @brief Hitecのパラメータ系コマンド1件分の処理(0xE9を1回).
class HtcParamOp : public MrdParamOpBase {
public:
  void begin(const MrdServoLineRef &a_line, int a_ix, uint8_t a_cmd, int16_t a_val) {
    target(a_line, a_ix, a_val);
    m_cmd = a_cmd;
    if (a_line.id[a_ix] < 0 || a_line.id[a_ix] > HTC_ID_MAX) {
      finish(MRD_PCMD_NO_REPLY);
      return;
    }
    if (a_cmd != 3 && a_cmd != 6 && a_cmd != 53) {
      finish(MRD_PCMD_NA);
      return;
    }
    if (a_cmd == 53) {
      mrd_htc_line(a_line.line).m_speed[a_ix] = uint8_t(constrain(int(a_val), 1, 255));
    }
  }

  bool next(MrdHduartXfer &a_x) {
    if (finished()) {
      return false;
    }
    HitecHalfDuplexClass &htc_tmp = mrd_htc_line(m_ln->line);
    mrd_htc_packet_make(m_tx, HTC_CMD_SPEED_POS, id(), htc_tmp.m_speed[m_ix]);
    mrd_servo_xfer(a_x, m_ln->bus, m_tx, 5, m_rx, HTC_REPLY_LEN);
    a_x.framed = true; // 返信はホストの区切りで受ける
    return true;
  }

  void complete(const MrdHduartXfer &a_x) {
    const int us_tmp = (int(m_rx[0]) << 8) | m_rx[1];
    if (!a_x.ok || us_tmp < HTC_READ_MIN || us_tmp > HTC_READ_MAX) {
      finish(MRD_PCMD_NO_REPLY);
    } else if (m_cmd == 6) {
      finish(deg_to_val(mrd_htc_us2deg(us_tmp, m_ln->trim[m_ix], m_ln->cw[m_ix])));
    } else {
      finish(m_cmd == 53 ? m_val : int16_t(1));
    }
  }

private:
  uint8_t m_cmd = 0;
  uint8_t m_tx[5];
  uint8_t m_rx[HTC_REPLY_LEN] = {0, 0};
};

/// @brief Hitecサーボ1系統分の1フレームの処理.
///        トルクONのサーボがあれば, 最大 SERVO_HTC_PER_FRAME 台に順番に位置を送る(返信は待たない).
///        全サーボがトルクOFFなら, 最大 SERVO_HTC_PER_FRAME 台の現在位置を順番に読む(角度値は送らない).
///        全台脱力(0xEF)はOFFになったときとトルク状態の送り直しの間隔ごとに送る(1台分の枠を使う).
class HtcLineStepper : public MrdParamLineStepper<HtcParamOp> {
public:
  /// @param a_line 系統の参照.
  /// @param a_all_mount パラメータ系コマンドのスロットを除かないマウント状態(パラメータ系コマンドの実行用).
  /// @param a_num 処理するサーボ数.
  /// @param a_htc この系統のHitecサーボクラス(順番と脱力の状態を保持).
  HtcLineStepper(const MrdServoLineRef &a_line, const int *a_all_mount, int a_num, HitecHalfDuplexClass &a_htc)
      : MrdParamLineStepper<HtcParamOp>(a_line, a_all_mount, a_num),
        m_htc(a_htc) {}

protected:
  bool next_main(MrdHduartXfer &a_x) override {
    if (m_first) {
      m_first = false;
      m_htc.m_sent_prev = m_htc.m_sent_cur;
      m_htc.m_sent_cur = 0;
      bool any_on_tmp = false;
      for (int i = 0; i < m_num; i++) {
        any_on_tmp = any_on_tmp || (valid(i) && m_ln.on(i));
      }
      const bool refresh_tmp = m_ln.st->torque_tick();
      m_reading = !any_on_tmp;
      if (m_reading) {
        for (int i = 0; i < m_num; i++) { // 読まないサーボは前回の値(読み出しの結果で上書きする)
          if (valid(i)) {
            mrd_servo_past_keep(m_ln, i);
          }
        }
        if (!m_htc.m_freed || refresh_tmp) {
          m_htc.m_freed = true;
          m_sent++;
          m_ix = -1;
          return prepare(a_x, mrd_htc_packet_make(m_tx, HTC_CMD_FREE, 0, 0), false);
        }
      } else {
        m_htc.m_freed = false;
      }
    }
    if (m_sent >= cfg.servo_htc_per_frame) {
      return false;
    }
    for (int n = 0; n < m_num; n++) {
      const int ix_tmp = (m_ln.st->read_cursor + n) % m_num;
      if (!valid(ix_tmp)) {
        continue;
      }
      if (m_reading) { // 脱力中: 現在位置を読む
        m_ln.st->read_cursor = (ix_tmp + 1) % m_num;
        m_sent++;
        m_ix = ix_tmp;
        return prepare(a_x, mrd_htc_packet_make(m_tx, HTC_CMD_SPEED_POS, uint8_t(m_ln.id[ix_tmp]), m_htc.m_speed[ix_tmp]), true);
      }
      // 動作中: 前のフレームで送ったサーボは送らない(各サーボ50Hzまで)
      if (!m_ln.on(ix_tmp) || (m_htc.m_sent_prev & (1u << ix_tmp))) {
        continue;
      }
      m_ln.st->read_cursor = (ix_tmp + 1) % m_num;
      m_htc.m_sent_cur |= (1u << ix_tmp);
      m_sent++;
      m_ix = -1;
      const uint16_t us_tmp = mrd_htc_deg2us(m_ln.tgt[ix_tmp], m_ln.trim[ix_tmp], m_ln.cw[ix_tmp]);
      return prepare(a_x, mrd_htc_packet_make(m_tx, uint8_t(m_ln.id[ix_tmp]), us_tmp >> 8, us_tmp & 0xFF), false);
    }
    return false;
  }

  /// @brief 読み出しの結果を反映する. 位置命令と脱力の返信は読まない(指令値をそのまま返す).
  void complete_main(const MrdHduartXfer &a_x) override {
    if (m_ix < 0) {
      return;
    }
    const int us_tmp = (int(m_rx[0]) << 8) | m_rx[1];
    const bool ok_tmp = a_x.ok && us_tmp >= HTC_READ_MIN && us_tmp <= HTC_READ_MAX;
    mrd_servo_read_apply(m_ln, m_ix, ok_tmp, ok_tmp ? mrd_htc_us2deg(us_tmp, m_ln.trim[m_ix], m_ln.cw[m_ix]) : 0.0f);
  }

private:
  bool valid(int a_ix) const { return m_ln.mount[a_ix] && m_ln.id[a_ix] >= 0 && m_ln.id[a_ix] <= HTC_ID_MAX; }

  /// @param a_read 返信を区切り受信で読むか(読み出し命令). falseなら返信枠の分だけ線を開放して待つ.
  bool prepare(MrdHduartXfer &a_x, size_t a_len, bool a_read) {
    a_x.bus = m_ln.bus;
    a_x.tx = m_tx;
    a_x.tx_len = a_len;
    a_x.rx = m_rx;
    a_x.rx_len = a_read ? HTC_REPLY_LEN : 0;
    a_x.framed = a_read;
    a_x.hold_us = a_read ? 0 : m_htc.reply_hold_us();
    return true;
  }

  HitecHalfDuplexClass &m_htc;
  bool m_first = true;
  bool m_reading = false; // このフレームは脱力中の読み出しか
  int m_sent = 0;         // このフレームで送ったパケット数
  int m_ix = -1;          // 直前に読み出したサーボのインデックス(-1:読み出し以外)
  uint8_t m_tx[5];
  uint8_t m_rx[HTC_REPLY_LEN] = {0, 0};
};

#endif // __MERIDIAN_SERVO_HITEC_HSR_H__
