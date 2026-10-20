#ifndef __MERIDIAN_SERVO_KONDO_ICS_H__
#define __MERIDIAN_SERVO_KONDO_ICS_H__

#include "config.h"
#include "main.h"
#include "mrd_servo.h" // 半二重UARTバスと共通処理. 本ファイルは mrd_servo.h から読み込まれる
#include "sv_util.h"   // 各プロトコル層の共通部品

#include <IcsBaseClass.h>

//==================================================================================================
//  KONDO ICS プロトコル層 (半二重UARTバス上で動作)
//==================================================================================================
//
// 対象: KRS-2552, KRS-3301, KRS-4031 等の ICS3.5 / 3.6 サーボ(近藤科学 ICS3.5/3.6 ソフトウェアマニュアル).
//
// 通信: 8E1(パリティ偶数). 通信速度は 115200 / 625000 / 1250000bps(サーボの設定に合わせる).
// 位置命令: 0x80+ID, 位置H(7bit), 位置L(7bit) の3バイト → 返信も3バイト(現在位置). 位置0は脱力.
// 1フレームの処理: 各サーボに位置命令を1回ずつ送り, 返信の現在位置を反映する(トルクOFFのサーボは脱力の命令).
// 位置値: 7500が中央(0deg), 3500〜11500 = ±135deg. 角度の変換は Meridian ライブラリ(mrd.Deg2Krs / mrd.Krs2Deg).
// 注意: 命令にチェックサムがないため, 他の種類のサーボと同じ系統に混在させない.

/// @brief IcsBaseClassの送受信をMrdHalfDuplexUartに委譲するICSクラス.
///        setPos, setFree, getKrrAllData等はIcsBaseClassのものをそのまま使用する.
class IcsHalfDuplexClass : public IcsBaseClass {
public:
  /// @brief コンストラクタ.
  /// @param a_bus 使用する半二重UARTバス.
  /// @param a_baud ICSの通信速度(115200, 625000, 1250000bps).
  IcsHalfDuplexClass(MrdHalfDuplexUart &a_bus, unsigned long a_baud)
      : m_bus(a_bus), m_baud(a_baud) {}

  /// @brief 通信速度を設定し直す(起動時に EEPROM の設定を反映するため). begin() の前に呼ぶ.
  void set_baud(unsigned long a_baud) { m_baud = a_baud; }

  /// @brief ICS用のフレーム形式(8E1)でバスを開始する.
  bool begin() { return m_bus.begin(m_baud, SERIAL_8E1); }

  bool synchronize(byte *a_tx_buf, byte a_tx_len, byte *a_rx_buf, byte a_rx_len) override {
    return m_bus.transfer(a_tx_buf, a_tx_len, a_rx_buf, a_rx_len);
  }

  /// @brief 半二重UARTバス(並行送受信用).
  MrdHalfDuplexUart &bus() { return m_bus; }

  /// @brief 系統の参照に渡す状態(ICSは位置命令の返信が現在位置のため, トルク状態の記憶は使わない).
  MrdServoLineState m_state;

private:
  MrdHalfDuplexUart &m_bus;
  unsigned long m_baud;
};

//==================================================================================================
//  KONDO ICSサーボ関連の処理
//==================================================================================================

/// @brief ICSサーボの応答(ポジション値)をエラー処理して角度に変換する.
/// @param a_val サーボから返ったポジション値(-1:返信なし)
/// @param a_tgt_past 前回のサーボの目標位置
/// @param a_trim サーボの補正値
/// @param a_cw サーボの回転方向補正値
/// @param a_err_cnt サーボのエラーカウント
/// @param a_stat サーボのステータス
/// @return 角度(degree)
float mrd_ics_result2deg(int a_val, float a_tgt_past, float a_trim, int a_cw, int &a_err_cnt,
                         uint16_t &a_stat) {
  int val_tmp = a_val;
  if (val_tmp == -1) { // サーボからの返信信号を受け取れなかった場合
    val_tmp = mrd.Deg2Krs(a_tgt_past, a_trim, a_cw);
    a_err_cnt++;
    if (a_err_cnt >= cfg.servo_lost_err_wait) { // 一定以上の連続エラーで通信不能とみなす
      a_err_cnt = cfg.servo_lost_err_wait;
      a_stat = 1;
    }
  } else {
    a_err_cnt = 0;
    a_stat = 0;
  }

  return mrd.Krs2Deg(val_tmp, a_trim, a_cw);
}

//------------------------------------------------------------------------------------
//  パラメータ系コマンド (番号と値の形式は tasks/servo_param_plan.md)
//------------------------------------------------------------------------------------
//
// 読み出し: 0xA0+ID, サブコマンド(1:ストレッチ, 2:スピード, 3:電流値, 4:温度値, 5:現在位置[ICS3.6]) → 0x20+ID, SC, 値
// 書き込み: 0xC0+ID, サブコマンド(1〜4), 値 → 0x40+ID, SC, 値. RAMのみで電源を切ると戻る(ソフトウェアマニュアルv2.9).
// EEPROM: 0xA0+ID, 0 → 0x20+ID, 0, 64バイト / 0xC0+ID, 0, 64バイト → 0x40+ID, 0. 1バイトに4bitずつ(上位が先).
//         64バイト一括でしか書けないため, 読んで1項目だけ変えて書き戻す. 読み出しはRAMの内容を返すため,
//         そのまま書き戻すとRAMの値(ストレッチ等)が保存される(35: ROM保存).

#define ICS_EEPROM_LEN        64 // EEPROMのデータ長(4bitずつ)
#define ICS_EEPROM_TIMEOUT_MS 50 // EEPROM書き込みの返信待ち(マニュアルに記載なし. 余裕を持たせる)

/// @brief 温度の表(ソフトウェアマニュアル: 30=100℃, 47=90℃, 60=80℃, 75=70℃, 87=60℃. 値が小さいほど高温).
static const int16_t ICS_TEMP_TABLE[][2] = {{30, 100}, {47, 90}, {60, 80}, {75, 70}, {87, 60}};

/// @brief ICSの温度値を℃にする(表の折れ線で補間, 表の外は端の傾きで延長).
inline int16_t mrd_ics_raw2temp(int a_raw) {
  int k = 0;
  while (k < 3 && a_raw > ICS_TEMP_TABLE[k + 1][0]) {
    k++;
  }
  const float r0_tmp = ICS_TEMP_TABLE[k][0], t0_tmp = ICS_TEMP_TABLE[k][1];
  const float r1_tmp = ICS_TEMP_TABLE[k + 1][0], t1_tmp = ICS_TEMP_TABLE[k + 1][1];
  return int16_t(lroundf(t0_tmp + (a_raw - r0_tmp) * (t1_tmp - t0_tmp) / (r1_tmp - r0_tmp)));
}

/// @brief ℃をICSの温度値にする(mrd_ics_raw2temp の逆. 1〜127).
inline int mrd_ics_temp2raw(int a_temp) {
  int k = 0;
  while (k < 3 && a_temp < ICS_TEMP_TABLE[k + 1][1]) {
    k++;
  }
  const float r0_tmp = ICS_TEMP_TABLE[k][0], t0_tmp = ICS_TEMP_TABLE[k][1];
  const float r1_tmp = ICS_TEMP_TABLE[k + 1][0], t1_tmp = ICS_TEMP_TABLE[k + 1][1];
  return constrain(int(lroundf(r0_tmp + (a_temp - t0_tmp) * (r1_tmp - r0_tmp) / (t1_tmp - t0_tmp))), 1, 127);
}

/// @brief ICSのパラメータ系コマンド1件分の処理.
///        サブコマンドの読み書きは1回, EEPROMの読み出しは1回, EEPROMの書き込みは読み出し → 書き込みの2回.
///        書き込みの結果は受け取ったVAL.
class IcsParamOp : public MrdParamOpBase {
public:
  void begin(const MrdServoLineRef &a_line, int a_ix, uint8_t a_cmd, int16_t a_val) {
    target(a_line, a_ix, a_val);
    if (a_line.id[a_ix] < IcsBaseClass::MIN_ID || a_line.id[a_ix] > IcsBaseClass::MAX_ID) {
      finish(MRD_PCMD_NO_REPLY);
      return;
    }
    switch (a_cmd) {
    case 3: // 応答確認(ストレッチを読む)
      set(SUB_READ, 1, CV_PING);
      break;
    case 4: // 目標位置(位置命令. トルクが入る)
      set(POS_WRITE, 0, CV_POS);
      break;
    case 6: // 現在位置(ICS3.6)
      set(SUB_READ, 5, CV_POS);
      break;
    case 12:
    case 13: // CW側の角度限界(上限パルス)
      set(a_cmd == 12 ? EEP_READ : EEP_WRITE, 16, CV_POS_NATIVE, 4);
      break;
    case 14:
    case 15: // CCW側の角度限界(下限パルス)
      set(a_cmd == 14 ? EEP_READ : EEP_WRITE, 20, CV_POS_NATIVE, 4);
      break;
    case 16:
    case 17:
    case 50:
    case 51: // ストレッチ(1〜127)
      set((a_cmd == 16 || a_cmd == 50) ? SUB_READ : SUB_WRITE, 1, CV_RAW);
      break;
    case 22: // 温度上限(EEPROM)
      set(EEP_READ, 28, CV_TEMP);
      break;
    case 23:
      set(SUB_WRITE, 4, CV_TEMP);
      break;
    case 24: // 電流上限(EEPROM, 0.1A)
      set(EEP_READ, 30, CV_CURRENT);
      break;
    case 25:
      set(SUB_WRITE, 3, CV_CURRENT);
      break;
    case 35: // ROM保存(EEPROMを読んでそのまま書き戻す)
      set(EEP_WRITE, 0, CV_SAVE);
      break;
    case 36:
      set(SUB_READ, 4, CV_TEMP);
      break;
    case 37:
      set(SUB_READ, 3, CV_CURRENT_NOW);
      break;
    case 52:
    case 53: // スピード(1〜127)
      set(a_cmd == 52 ? SUB_READ : SUB_WRITE, 2, CV_RAW);
      break;
    default: {
      // 50以降のEEPROMの項目(2バイト分の4bit = 1バイトの値)
      const int addr_tmp = eeprom_addr(a_cmd);
      if (addr_tmp < 0) {
        finish(MRD_PCMD_NA);
        return;
      }
      set((a_cmd % 2 == 0) ? EEP_READ : EEP_WRITE, uint8_t(addr_tmp), CV_RAW);
      break;
    }
    }
  }

  bool next(MrdHduartXfer &a_x) {
    if (finished()) {
      return false;
    }
    const uint8_t id_tmp = id();
    switch (m_step) {
    case SUB_READ:
      m_tx[0] = 0xA0 + id_tmp;
      m_tx[1] = m_sc;
      mrd_servo_xfer(a_x, m_ln->bus, m_tx, 2, m_rx, m_sc == 5 ? 4 : 3);
      return true;
    case SUB_WRITE:
      m_tx[0] = 0xC0 + id_tmp;
      m_tx[1] = m_sc;
      m_tx[2] = uint8_t(sub_value());
      mrd_servo_xfer(a_x, m_ln->bus, m_tx, 3, m_rx, 3);
      return true;
    case POS_WRITE: {
      const int pos_tmp = mrd.Deg2Krs(m_val * 0.01f, m_ln->trim[m_ix], m_ln->cw[m_ix]);
      m_tx[0] = 0x80 + id_tmp;
      m_tx[1] = (pos_tmp >> 7) & 0x7F;
      m_tx[2] = pos_tmp & 0x7F;
      mrd_servo_xfer(a_x, m_ln->bus, m_tx, 3, m_rx, 3);
      return true;
    }
    case EEP_READ:
    case EEP_WRITE: // まず64バイトを読む
      m_tx[0] = 0xA0 + id_tmp;
      m_tx[1] = 0;
      mrd_servo_xfer(a_x, m_ln->bus, m_tx, 2, m_rx, 2 + ICS_EEPROM_LEN);
      return true;
    case EEP_STORE: // 書き戻す(m_rx[2..65]を変更済み)
      m_tx[0] = 0xC0 + id_tmp;
      m_tx[1] = 0;
      memcpy(&m_tx[2], &m_rx[2], ICS_EEPROM_LEN);
      mrd_servo_xfer(a_x, m_ln->bus, m_tx, 2 + ICS_EEPROM_LEN, m_rx, 2, ICS_EEPROM_TIMEOUT_MS);
      return true;
    default:
      return false;
    }
  }

  void complete(const MrdHduartXfer &a_x) {
    const uint8_t id_tmp = id();
    switch (m_step) {
    case SUB_READ: {
      if (!a_x.ok || m_rx[0] != 0x20 + id_tmp || m_rx[1] != m_sc) {
        finish(MRD_PCMD_NO_REPLY);
      } else if (m_cv == CV_PING) {
        finish(1);
      } else if (m_cv == CV_POS) {
        const int pos_tmp = ((m_rx[2] << 7) & 0x3F80) + (m_rx[3] & 0x7F);
        finish(deg_to_val(mrd.Krs2Deg(pos_tmp, m_ln->trim[m_ix], m_ln->cw[m_ix])));
      } else {
        finish(from_raw(m_rx[2]));
      }
      break;
    }
    case SUB_WRITE:
      finish((a_x.ok && m_rx[0] == 0x40 + id_tmp && m_rx[1] == m_sc) ? m_val : MRD_PCMD_NO_REPLY);
      break;
    case POS_WRITE:
      finish(a_x.ok ? m_val : MRD_PCMD_NO_REPLY);
      break;
    case EEP_READ:
    case EEP_WRITE: {
      if (!a_x.ok || m_rx[0] != 0x20 + id_tmp || m_rx[1] != 0) {
        finish(MRD_PCMD_NO_REPLY);
        break;
      }
      uint8_t *eep_tmp = &m_rx[2];
      if (m_step == EEP_READ) {
        finish(from_raw(get_field(eep_tmp)));
        break;
      }
      if (m_cv != CV_SAVE) {
        put_field(eep_tmp, to_raw());
      }
      m_step = EEP_STORE;
      break;
    }
    case EEP_STORE:
      finish((a_x.ok && m_rx[0] == 0x40 + id_tmp && m_rx[1] == 0) ? (m_cv == CV_SAVE ? int16_t(1) : m_val)
                                                                  : MRD_PCMD_NO_REPLY);
      break;
    default:
      break;
    }
  }

private:
  enum Step : uint8_t { SUB_READ = 1,
                        SUB_WRITE,
                        POS_WRITE,
                        EEP_READ,
                        EEP_WRITE,
                        EEP_STORE };
  enum Conv : uint8_t { CV_RAW,
                        CV_PING,
                        CV_POS,
                        CV_POS_NATIVE,
                        CV_TEMP,
                        CV_CURRENT,
                        CV_CURRENT_NOW,
                        CV_SAVE };

  void set(Step a_step, uint8_t a_sc_or_addr, Conv a_cv, uint8_t a_nibbles = 2) {
    m_step = a_step;
    m_sc = a_sc_or_addr;
    m_addr = a_sc_or_addr;
    m_cv = a_cv;
    m_nibbles = a_nibbles;
  }

  /// @brief 50以降のEEPROMの項目のアドレス(4bit単位の位置). 対応しなければ-1.
  static int eeprom_addr(uint8_t a_cmd) {
    switch (a_cmd) {
    case 54:
    case 55:
      return 6; // パンチ(0〜10)
    case 56:
    case 57:
      return 8; // デッドバンド(0〜5)
    case 58:
    case 59:
      return 10; // ダンピング(1〜255)
    case 62:
    case 63:
      return 58; // 特性変化ストレッチ1(2〜254)
    case 64:
    case 65:
      return 60; // 特性変化ストレッチ2
    case 66:
    case 67:
      return 62; // 特性変化ストレッチ3
    case 68:
    case 69:
      return 12; // セーフタイマー(10〜255)
    case 70:
    case 71:
      return 50; // レスポンス(1〜5)
    default:
      return -1;
    }
  }

  /// @brief EEPROMの項目を読む(4bitずつ, 上位が先).
  int get_field(const uint8_t *a_eep) const {
    int v_tmp = 0;
    for (int i = 0; i < m_nibbles; i++) {
      v_tmp = (v_tmp << 4) | (a_eep[m_addr + i] & 0x0F);
    }
    return v_tmp;
  }

  /// @brief EEPROMの項目を書く.
  void put_field(uint8_t *a_eep, int a_val) const {
    for (int i = 0; i < m_nibbles; i++) {
      a_eep[m_addr + i] = (a_val >> (4 * (m_nibbles - 1 - i))) & 0x0F;
    }
  }

  /// @brief 生値をVALにする.
  int16_t from_raw(int a_raw) const {
    switch (m_cv) {
    case CV_POS_NATIVE:
      return deg_to_val(mrd.Krs2Deg(a_raw, 0.0f, 1));
    case CV_TEMP:
      return mrd_ics_raw2temp(a_raw);
    case CV_CURRENT:
      return int16_t(a_raw * 100);
    case CV_CURRENT_NOW: // 0〜63:正転, 64〜127:逆転
      return int16_t(((a_raw & 0x40) ? a_raw - 64 : a_raw) * 100);
    default:
      return int16_t(a_raw);
    }
  }

  /// @brief VALを生値にする(EEPROMの項目).
  int to_raw() const {
    if (m_cv == CV_POS_NATIVE) {
      return mrd.Deg2Krs(m_val * 0.01f, 0.0f, 1);
    }
    return constrain(int(m_val), 0, (1 << (4 * m_nibbles)) - 1);
  }

  /// @brief VALをサブコマンドの書き込み値にする.
  int sub_value() const {
    if (m_cv == CV_TEMP) {
      return mrd_ics_temp2raw(m_val);
    }
    if (m_cv == CV_CURRENT) {
      return constrain(int(lroundf(m_val * 0.01f)), 1, 63);
    }
    return constrain(int(m_val), 1, 127);
  }

  uint8_t m_sc = 0;      // サブコマンド
  uint8_t m_addr = 0;    // EEPROMの項目の位置(4bit単位)
  uint8_t m_nibbles = 2; // EEPROMの項目の長さ(4bit単位)
  Conv m_cv = CV_RAW;
  uint8_t m_tx[2 + ICS_EEPROM_LEN];
  uint8_t m_rx[2 + ICS_EEPROM_LEN];
};

/// @brief 系統のICSサーボクラス.
inline IcsHalfDuplexClass &mrd_ics_line(int a_line) { return (a_line == L) ? ics_l : ics_r; }

/// @brief 系統の参照を作る.
MrdServoLineRef mrd_ics_line_ref(UartLine a_line, Meridim90Union &a_meridim, ServoParam &a_sv) {
  return mrd_servo_line_ref(a_line, KOICS3, mrd_ics_line(a_line).bus(), mrd_ics_line(a_line).m_state, a_meridim, a_sv);
}

/// @brief ICSサーボ1系統分の1フレームの処理を, 1回ずつの送受信に分けて進める.
///        コマンド1のサーボは目標位置へ(setPos と同じ), それ以外は脱力(setFree と同じ). 返信の現在位置を反映する.
///        パラメータ系コマンドのスロットは通常の処理から外し(保持), 最後にパラメータ系コマンドを実行する.
class IcsLineStepper : public MrdParamLineStepper<IcsParamOp> {
public:
  /// @param a_line 系統の参照.
  /// @param a_all_mount パラメータ系コマンドのスロットを除かないマウント状態(パラメータ系コマンドの実行用).
  /// @param a_num 処理するサーボ数.
  IcsLineStepper(const MrdServoLineRef &a_line, const int *a_all_mount, int a_num)
      : MrdParamLineStepper<IcsParamOp>(a_line, a_all_mount, a_num) {}

protected:
  bool next_main(MrdHduartXfer &a_x) override {
    m_ix = m_ln.next_mounted(m_ix, m_num);
    if (m_ix >= m_num) {
      return false;
    }
    a_x.bus = m_ln.bus;
    a_x.rx = m_rx;
    a_x.rx_len = sizeof(m_rx);
    const int id_tmp = m_ln.id[m_ix];
    if (id_tmp < IcsBaseClass::MIN_ID || id_tmp > IcsBaseClass::MAX_ID) {
      a_x.tx_len = 0; // 範囲外のIDは送らず失敗扱い(setPos/setFreeと同じ)
      return true;
    }
    m_tx[0] = 0x80 + id_tmp; // CMD
    if (m_ln.on(m_ix)) {     // コマンドが1ならPos指定
      const int pos_tmp = mrd.Deg2Krs(m_ln.tgt[m_ix], m_ln.trim[m_ix], m_ln.cw[m_ix]);
      m_tx[1] = (pos_tmp >> 7) & 0x7F; // POS_H
      m_tx[2] = pos_tmp & 0x7F;        // POS_L
    } else {                           // コマンドが0等なら脱力して値を取得
      m_tx[1] = 0;
      m_tx[2] = 0;
    }
    a_x.tx = m_tx;
    a_x.tx_len = sizeof(m_tx);
    return true;
  }

  void complete_main(const MrdHduartXfer &a_x) override {
    // 返信の先頭は送ったCMDの最上位bitを落とした値(= ID). 違えば他のサーボの返信やずれた受信として失敗にする
    const bool ok_tmp = a_x.ok && m_rx[0] == (m_tx[0] & 0x7F);
    const int val_tmp = ok_tmp ? ((m_rx[1] << 7) & 0x3F80) + (m_rx[2] & 0x007F) : IcsBaseClass::ICS_FALSE;
    m_ln.tgt[m_ix] = mrd_ics_result2deg(val_tmp, m_ln.tgt_past[m_ix], m_ln.trim[m_ix], m_ln.cw[m_ix], m_ln.err[m_ix],
                                        m_ln.stat[m_ix]);
  }

private:
  int m_ix = -1;
  uint8_t m_tx[3];
  uint8_t m_rx[3];
};

#endif // __MERIDIAN_SERVO_KONDO_ICS_H__
